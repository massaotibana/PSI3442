# my_drone_mission_cpp

Missão **OFFBOARD** com PX4/ROS 2: visita os pontos **A → B → C** (hold 10s) e **retorna para Home** quando a bateria cai abaixo de **90%**. Ao pousar, **aguarda “recarregar”** até **≥95%** e **retoma** a missão a partir do próximo ponto pendente. Ao final, **retorna e pousa**.

## Requisitos

- ROS 2 (Jazzy) e `px4_msgs` instalados no workspace
- PX4 SITL rodando com uXRCE-DDS Client
- Micro XRCE-DDS Agent ativo (porta padrão 8888)
- Gazebo/Ignition conforme seu ambiente

## Build

```bash
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_drone_mission_cpp --event-handlers console_direct+

Fluxo da missão:
IDLE → START_OFFBOARD → SET_OFFBOARD → ARM → TAKEOFF
→ NAVIGATE(A/B/C) ↔ HOLD(10s)
↘ (SOC<90%) RETURN_HOME_CHARGE → LAND_FOR_CHARGE → RECHARGE_WAIT (SOC≥95%)
→ RESUME_START_OFFBOARD → RESUME_SET_OFFBOARD → RESUME_ARM → RESUME_TAKEOFF
→ NAVIGATE/HOLD (continua)
→ FINAL_RETURN → FINAL_LAND → DONE
