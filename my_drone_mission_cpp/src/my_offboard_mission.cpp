#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;

struct Vec3
{
    float x, y, z;
}; // NED (z<0 = acima do solo)

class OffboardMission : public rclcpp::Node
{
public:
    OffboardMission() : Node("offboard_mission")
    {
        offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // QoS
        auto qos_batt = rclcpp::QoS(10).best_effort().transient_local();    // bateria: TRANSIENT_LOCAL
        auto qos_tel = rclcpp::QoS(10).best_effort().durability_volatile(); // telemetria: VOLATILE

        // Battery (prioriza remaining -> volt_based -> tensão)
        auto battery_cb = [this](const BatteryStatus::SharedPtr m)
        {
            const uint64_t now = now_us_wall();
            if (m->timestamp != 0 && now > m->timestamp && (now - m->timestamp) > 2'000'000)
                return; // descarte latched >2s

            float soc = NAN;

            // 1) SOC do simulador (drena conforme SIM_BAT_*)
            if (std::isfinite(m->remaining) && m->remaining >= 0.f && m->remaining <= 1.f)
            {
                soc = m->remaining;
            }
            // 2) Fallback: SOC baseado em tensão do PX4
            else if (std::isfinite(m->volt_based_soc_estimate) &&
                     m->volt_based_soc_estimate >= 0.f && m->volt_based_soc_estimate <= 1.f)
            {
                soc = m->volt_based_soc_estimate;
            }
            // 3) Último recurso: converter tensão absoluta em %
            else if (std::isfinite(m->voltage_v) && m->voltage_v > 0.f)
            {
                const int cells = (m->cell_count > 0) ? m->cell_count : 4;
                const float V_FULL = 4.2f * cells;
                const float V_EMPTY = 3.5f * cells;
                float pct = (m->voltage_v - V_EMPTY) / (V_FULL - V_EMPTY);
                soc = std::clamp(pct, 0.f, 1.f);
            }

            if (std::isfinite(soc))
            {
                batt_percent_ = soc * 100.f;
                have_batt_ = true;
            }
        };
        batt_sub_v1_ = create_subscription<BatteryStatus>("/fmu/out/battery_status_v1", qos_batt, battery_cb);
        batt_sub_ = create_subscription<BatteryStatus>("/fmu/out/battery_status", qos_batt, battery_cb);

        // Posição/odometria
        loc_sub_ = create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_tel,
                                                             [this](VehicleLocalPosition::SharedPtr m)
                                                             {
                                                                 x_ = m->x;
                                                                 y_ = m->y;
                                                                 z_ = m->z;
                                                                 pos_ok_ = std::isfinite(x_) && std::isfinite(y_) && std::isfinite(z_);
                                                             });
        odom_sub_ = create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos_tel,
                                                         [this](VehicleOdometry::SharedPtr m)
                                                         {
                                                             x_ = m->position[0];
                                                             y_ = m->position[1];
                                                             z_ = m->position[2];
                                                             vz_ = m->velocity[2];
                                                             pos_ok_ = std::isfinite(x_) && std::isfinite(y_) && std::isfinite(z_);
                                                         });

        land_sub_ = create_subscription<VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos_tel,
                                                             [this](VehicleLandDetected::SharedPtr m)
                                                             {
                                                                 landed_ = m->landed;
                                                                 maybe_landed_ = m->maybe_landed;
                                                                 if (landed_)
                                                                     last_landed_true_wall_ = now_s_wall();
                                                             });

        // Missão
        HOME_ = {0.f, 0.f, -2.f};
        points_ = {{15.f, 0.f, -2.f}, {15.f, 15.f, -2.f}, {0.f, 15.f, -2.f}};
        visited_.assign(points_.size(), false);

        hold_sec_ = 10.0;
        batt_rtl_pct_ = 90.0f;    // retornar p/ recharge
        batt_resume_pct_ = 95.0f; // retomar após ≥95%
        tol_m_ = 0.8f;

        enter_state(State::IDLE);

        main_timer_ = create_wall_timer(100ms, std::bind(&OffboardMission::tick, this));
        mon_timer_ = create_wall_timer(1000ms, std::bind(&OffboardMission::monitor, this));
        RCLCPP_INFO(get_logger(), "Mission READY (RTL<%.1f%%, resume ≥%.1f%%, hold=%.0fs).",
                    batt_rtl_pct_, batt_resume_pct_, hold_sec_);
    }

private:
    enum class State
    {
        IDLE,
        START_OFFBOARD,
        SET_OFFBOARD,
        ARM,
        TAKEOFF,
        NAVIGATE,
        HOLD,
        RETURN_HOME_CHARGE,
        LAND_FOR_CHARGE,
        RECHARGE_WAIT,
        RESUME_START_OFFBOARD,
        RESUME_SET_OFFBOARD,
        RESUME_ARM,
        RESUME_TAKEOFF,
        FINAL_RETURN,
        FINAL_LAND,
        DONE
    };

    // Tempo/timestamps
    inline uint64_t now_us_wall() const { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }
    inline double now_s_wall() const { return get_clock()->now().seconds(); }
    inline double since_enter_s() const { return std::chrono::duration<double>(SteadyClock::now() - t_enter_steady_).count(); }

    void enter_state(State s)
    {
        state_ = s;
        t_enter_steady_ = SteadyClock::now();
        pre_count_ = 0;
    }

    bool low_batt() const { return have_batt_ && batt_percent_ < batt_rtl_pct_; }
    bool charged() const { return have_batt_ && batt_percent_ >= batt_resume_pct_; }
    bool all_visited() const
    {
        return std::all_of(visited_.begin(), visited_.end(), [](bool v)
                           { return v; });
    }
    int next_unvisited_index() const
    {
        for (size_t i = 0; i < visited_.size(); ++i)
            if (!visited_[i])
                return static_cast<int>(i);
        return -1;
    }
    bool near(const Vec3 &g) const
    {
        if (!pos_ok_)
            return false;
        float dx = x_ - g.x, dy = y_ - g.y, dz = z_ - g.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= tol_m_;
    }

    const char *sname(State s) const
    {
        switch (s)
        {
        case State::IDLE:
            return "IDLE";
        case State::START_OFFBOARD:
            return "START_OFFBOARD";
        case State::SET_OFFBOARD:
            return "SET_OFFBOARD";
        case State::ARM:
            return "ARM";
        case State::TAKEOFF:
            return "TAKEOFF";
        case State::NAVIGATE:
            return "NAVIGATE";
        case State::HOLD:
            return "HOLD";
        case State::RETURN_HOME_CHARGE:
            return "RETURN_HOME_CHARGE";
        case State::LAND_FOR_CHARGE:
            return "LAND_FOR_CHARGE";
        case State::RECHARGE_WAIT:
            return "RECHARGE_WAIT";
        case State::RESUME_START_OFFBOARD:
            return "RESUME_START_OFFBOARD";
        case State::RESUME_SET_OFFBOARD:
            return "RESUME_SET_OFFBOARD";
        case State::RESUME_ARM:
            return "RESUME_ARM";
        case State::RESUME_TAKEOFF:
            return "RESUME_TAKEOFF";
        case State::FINAL_RETURN:
            return "FINAL_RETURN";
        case State::FINAL_LAND:
            return "FINAL_LAND";
        case State::DONE:
            return "DONE";
        }
        return "?";
    }

    std::string batt_str() const
    {
        if (have_batt_ && std::isfinite(batt_percent_))
        {
            char b[16];
            std::snprintf(b, sizeof(b), "%.1f%%", batt_percent_);
            return std::string(b);
        }
        return "n/a";
    }

    // Publicadores/Comandos
    void pub_offboard()
    {
        OffboardControlMode m{};
        m.position = true;
        m.velocity = false;
        m.acceleration = false;
        m.attitude = false;
        m.body_rate = false;
        m.timestamp = now_us_wall();
        offboard_pub_->publish(m);
    }
    void pub_sp(const Vec3 &p, float yaw = 0.f)
    {
        TrajectorySetpoint t{};
        t.position = {p.x, p.y, p.z};
        t.yaw = yaw;
        t.timestamp = now_us_wall();
        traj_pub_->publish(t);
    }
    void cmd(uint16_t c, float p1 = 0.f, float p2 = 0.f)
    {
        VehicleCommand v{};
        v.param1 = p1;
        v.param2 = p2;
        v.command = c;
        v.target_system = 1;
        v.target_component = 1;
        v.source_system = 1;
        v.source_component = 1;
        v.from_external = true;
        v.timestamp = now_us_wall();
        cmd_pub_->publish(v);
    }
    void set_offboard() { cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); }
    void arm() { cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void disarm() { cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); }

    // Pre-stream de ~1s (10 amostras a 10Hz)
    bool prestream_to(const Vec3 &target)
    {
        pub_sp(target);
        return (++pre_count_ >= 10);
    }

    // Monitor
    void monitor()
    {
        RCLCPP_INFO(get_logger(), "[%s] batt=%s pos=(%.2f,%.2f,%.2f) vz=%.2f",
                    sname(state_), batt_str().c_str(), x_, y_, z_, vz_);
    }

    // FSM
    void tick()
    {
        pub_offboard();

        // Aguarda posição e inicia ciclo de decolagem
        if (state_ == State::IDLE)
        {
            if (!pos_ok_ && since_enter_s() < 3.0)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Aguardando posicao local valida...");
                return;
            }
            enter_state(State::START_OFFBOARD);
            return;
        }

        // Bateria baixa durante missão => volta para recarregar
        if (!all_visited() &&
            (state_ == State::NAVIGATE || state_ == State::HOLD) &&
            low_batt() &&
            state_ != State::RETURN_HOME_CHARGE && state_ != State::LAND_FOR_CHARGE && state_ != State::RECHARGE_WAIT)
        {
            RCLCPP_WARN(get_logger(), "Battery low (%s) -> RETURN_HOME_CHARGE", batt_str().c_str());
            enter_state(State::RETURN_HOME_CHARGE);
        }

        switch (state_)
        {
        // ======= Decolagem inicial =======
        case State::START_OFFBOARD:
            if (prestream_to(HOME_))
            {
                enter_state(State::SET_OFFBOARD);
            }
            break;
        case State::SET_OFFBOARD:
            set_offboard();
            enter_state(State::ARM);
            break;
        case State::ARM:
            arm();
            enter_state(State::TAKEOFF);
            break;
        case State::TAKEOFF:
            pub_sp(HOME_); // sobe até z=-2 em HOME
            if (near(HOME_))
            {
                idx_ = next_unvisited_index();
                enter_state(idx_ >= 0 ? State::NAVIGATE : State::FINAL_RETURN);
            }
            break;

        // ======= Missão A/B/C =======
        case State::NAVIGATE:
        {
            const Vec3 &tgt = points_[idx_];
            pub_sp(tgt);
            if (near(tgt))
            {
                enter_state(State::HOLD);
                hold_start_ = SteadyClock::now();
            }
        }
        break;
        case State::HOLD:
        {
            const Vec3 &tgt = points_[idx_];
            pub_sp(tgt);
            auto held = std::chrono::duration<double>(SteadyClock::now() - hold_start_).count();
            if (held >= hold_sec_)
            {
                visited_[idx_] = true;
                int nxt = next_unvisited_index();
                if (nxt >= 0)
                {
                    idx_ = nxt;
                    enter_state(State::NAVIGATE);
                }
                else
                {
                    enter_state(State::FINAL_RETURN);
                }
            }
        }
        break;

        // ======= Volta para recarregar =======
        case State::RETURN_HOME_CHARGE:
            pub_sp(HOME_);
            if (near(HOME_))
            {
                enter_state(State::LAND_FOR_CHARGE);
                cmd(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
                land_start_wall_ = now_s_wall();
            }
            break;
        case State::LAND_FOR_CHARGE:
        {
            Vec3 land{HOME_.x, HOME_.y, 0.0f};
            pub_sp(land);
            bool landed_stable = landed_ && (now_s_wall() - last_landed_true_wall_ >= 1.0);
            bool touch_stable = (std::fabs(z_) < 0.05f && std::fabs(vz_) < 0.2f && since_enter_s() >= 1.5);
            bool timeout = (now_s_wall() - land_start_wall_ > 20.0);
            if (landed_stable || touch_stable || timeout)
            {
                disarm(); // desarma para retomar corretamente depois
                enter_state(State::RECHARGE_WAIT);
            }
        }
        break;
        case State::RECHARGE_WAIT:
            pub_sp({HOME_.x, HOME_.y, 0.0f});
            if (charged())
            {
                RCLCPP_INFO(get_logger(), "Recharged: batt=%s -> resume", batt_str().c_str());
                enter_state(State::RESUME_START_OFFBOARD);
            }
            else
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Recharging... batt=%s (alvo: %.1f%%). Ajuste SIM_BAT_MIN_PCT / reinicie SITL p/ subir SOC.",
                                     batt_str().c_str(), batt_resume_pct_);
            }
            break;

        // ======= Ciclo de decolagem para retomar =======
        case State::RESUME_START_OFFBOARD:
            if (prestream_to(HOME_))
            {
                enter_state(State::RESUME_SET_OFFBOARD);
            }
            break;
        case State::RESUME_SET_OFFBOARD:
            set_offboard();
            enter_state(State::RESUME_ARM);
            break;
        case State::RESUME_ARM:
            arm();
            enter_state(State::RESUME_TAKEOFF);
            break;
        case State::RESUME_TAKEOFF:
            pub_sp(HOME_);
            if (near(HOME_))
            {
                int nxt = next_unvisited_index();
                enter_state(nxt >= 0 ? State::NAVIGATE : State::FINAL_RETURN);
                if (nxt >= 0)
                    idx_ = nxt;
            }
            break;

        // ======= Encerramento =======
        case State::FINAL_RETURN:
            pub_sp(HOME_);
            if (near(HOME_))
            {
                enter_state(State::FINAL_LAND);
                cmd(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
                land_start_wall_ = now_s_wall();
            }
            break;
        case State::FINAL_LAND:
        {
            Vec3 land{HOME_.x, HOME_.y, 0.0f};
            pub_sp(land);
            bool landed_stable = landed_ && (now_s_wall() - last_landed_true_wall_ >= 1.0);
            bool touch_stable = (std::fabs(z_) < 0.05f && std::fabs(vz_) < 0.2f && since_enter_s() >= 1.5);
            bool timeout = (now_s_wall() - land_start_wall_ > 20.0);
            if (landed_stable || touch_stable || timeout)
            {
                disarm();
                enter_state(State::DONE);
            }
        }
        break;

        case State::DONE:
            pub_sp({x_, y_, z_});
            break;
        }
    }

    // pubs/subs
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<BatteryStatus>::SharedPtr batt_sub_;
    rclcpp::Subscription<BatteryStatus>::SharedPtr batt_sub_v1_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr loc_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;

    // timers
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr mon_timer_;

    // estado
    State state_;
    SteadyClock::time_point t_enter_steady_{};
    uint32_t pre_count_{0};

    // bateria
    bool have_batt_{false};
    float batt_percent_{NAN};

    // pose/vel
    bool pos_ok_{false};
    float x_{0.f}, y_{0.f}, z_{0.f};
    float vz_{NAN};

    // aterragem
    bool landed_{false}, maybe_landed_{false};
    double last_landed_true_wall_{-1.0};
    double land_start_wall_{-1.0};

    // missão
    Vec3 HOME_;
    std::vector<Vec3> points_;
    std::vector<bool> visited_;
    int idx_{-1};
    SteadyClock::time_point hold_start_{};

    // params
    double hold_sec_;
    float batt_rtl_pct_;
    float batt_resume_pct_;
    float tol_m_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardMission>());
    rclcpp::shutdown();
    return 0;
}
