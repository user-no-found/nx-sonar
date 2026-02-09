# ROS1 到 ROS2 桥接说明

当前链路为：PC 运行 ROS1 UUVSim，NX 运行 ROS2 感知节点。两端必须通过桥接层互通。

## 1. 网络与环境变量
确保 PC 与 NX 在同一网段，`ROS_DOMAIN_ID` 一致。

PC（ROS1 侧）：
```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>
export ROS_DOMAIN_ID=30
```

NX（ROS2 侧）：
```bash
export ROS_DOMAIN_ID=30
```

## 2. 在 PC 启动 UUVSim 与桥接
终端 A：
```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 B：
```bash
source /opt/ros/noetic/setup.bash
# 在此启动 uuvsim 对应 launch
```

终端 C：
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

## 3. 在 NX 启动统一管线
使用 UUVSim 适配器启动：
```bash
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=uuvsim
```

## 4. 校验话题链路
在 NX 上检查：
```bash
ros2 topic echo /sonar/std/frame/header
ros2 topic echo /sonar/detections/header
```

## 5. 切换到真实声呐
不改推理节点，仅切换适配器：
```bash
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=real
```
