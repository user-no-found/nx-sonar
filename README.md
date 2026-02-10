# 启动说明

本文件放在仓库根目录，用于说明多机启动顺序。默认仓库路径为 `~/nx-sonar-uuvsim`。

## TCP 网关

### 启动顺序
1. PC（ROS1）启动 Gazebo 水下世界。
2. PC（ROS1）启动带声呐的 ROV。
3. PC（ROS1）启动静态 TF 修复。
4. NX（ROS2 Humble）启动感知管线（`adapter_type:=tcp`）。
5. PC（ROS1）启动 LaserScan TCP 发送端。
6. NX 检查话题是否有数据。

### 具体命令
PC 终端1（Gazebo 水下世界）：
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uuv_gazebo_worlds herkules_ship_wreck.launch
```

PC 终端2（带声呐的 ROV）：
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uuv_gazebo rexrov_sonar.launch
```

PC 终端3（TF 修复，必需）：
```bash
source ~/catkin_ws/devel/setup.bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 rexrov/base_link sonar_link 100
```

NX 终端C：
```bash
cd ~/nx-sonar-uuvsim
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=tcp
```

PC 终端D：
```bash
cd ~/nx-sonar-uuvsim
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
