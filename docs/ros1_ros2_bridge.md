# ROS1 to ROS2 Bridge Guide

This project uses ROS1 UUVSim on PC and ROS2 perception on NX. A bridge layer is required.

## 1. Network and Environment
Set both machines to the same LAN and ROS domain.

On PC (ROS1 side):
```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>
export ROS_DOMAIN_ID=30
```

On NX (ROS2 side):
```bash
export ROS_DOMAIN_ID=30
```

## 2. Start UUVSim and ROS1 Bridge on PC
Terminal A:
```bash
source /opt/ros/noetic/setup.bash
roscore
```

Terminal B:
```bash
source /opt/ros/noetic/setup.bash
# start uuvsim launch here
```

Terminal C:
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

## 3. Start NX Pipeline
Run launch on NX with uuvsim adapter:
```bash
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=uuvsim
```

## 4. Verify Topic Flow
On NX:
```bash
ros2 topic echo /sonar/std/frame/header
ros2 topic echo /sonar/detections/header
```

## 5. Real Sonar Switch
Switch to real sonar adapter without changing inference node:
```bash
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=real
```
