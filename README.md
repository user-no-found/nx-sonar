# 启动说明

本文件放在仓库根目录，用于说明多机启动顺序。默认仓库路径为 `~/nx-sonar`。

## 方案A（推荐）：TCP 网关（不使用 ros1_bridge）

### 启动顺序
1. PC（ROS1）启动 `roscore`。
2. PC（ROS1）启动 UUVSim。
3. NX（ROS2）启动感知管线（`adapter_type:=tcp`）。
4. PC（ROS1）启动 LaserScan TCP 发送端。
5. NX 检查话题是否有数据。

### 具体命令
PC 终端A：
```bash
source /opt/ros/noetic/setup.bash
roscore
```

PC 终端B：
```bash
source /opt/ros/noetic/setup.bash
roslaunch <uuv包名> <launch文件>.launch
```

NX 终端C：
```bash
cd ~/nx-sonar
source /opt/ros/<你的ROS2版本>/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=tcp
```

PC 终端D：
```bash
cd ~/nx-sonar
source /opt/ros/noetic/setup.bash
python3 ros1/tools/ros1_gateway/laser_scan_tcp_sender.py \
  _server_host:=<NX_IP> \
  _server_port:=29001 \
  _scan_topic:=/rexrov/sonar
```

NX 检查：
```bash
ros2 topic hz /sonar/image_raw
ros2 topic echo /sonar/std/frame/header
ros2 topic echo /sonar/detections/header
```

## 方案B（可选）：ros1_bridge

仅在 PC 同时具备 ROS1+ROS2 环境时使用。

PC 桥接终端：
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=30
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

NX 启动：
```bash
cd ~/nx-sonar
source /opt/ros/<你的ROS2版本>/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=uuvsim
```

## 停止顺序
先停发送端/桥接，再停 NX 感知，最后停 UUVSim 和 `roscore`。
