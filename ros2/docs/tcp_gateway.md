# 自定义 TCP 网关使用说明（不使用 ros1_bridge）

适用场景：PC 仅 ROS1，NX 仅 ROS2，通过 TCP 传输声呐数据。

## 链路结构
`ROS1 /rexrov/sonar(LaserScan) -> TCP 发送端 -> TCP 接收端 -> /sonar/image_raw(Image) -> 适配器 -> YOLO`

## 1. NX 侧启动（ROS2）
在 NX 上启动管线，并启用 `tcp` 模式：

```bash
source /opt/ros/<你的ROS2版本>/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch sonar_bringup pipeline.launch.py adapter_type:=tcp
```

默认监听地址与端口：
- `bind_host: 0.0.0.0`
- `bind_port: 29001`

可在 `ros2/src/sonar_bringup/config/pipeline.yaml` 中修改。

## 2. PC 侧启动发送端（ROS1）
先启动 `roscore` 与 UUVSim 后，再运行发送脚本：

```bash
cd ~/nx-sonar
source /opt/ros/noetic/setup.bash
python3 ros1/tools/ros1_gateway/laser_scan_tcp_sender.py \
  _server_host:=<NX_IP> \
  _server_port:=29001 \
  _scan_topic:=/rexrov/sonar
```

## 3. 连通性检查
NX 侧检查：

```bash
ros2 topic hz /sonar/image_raw
ros2 topic echo /sonar/std/frame/header
ros2 topic echo /sonar/detections/header
```

## 4. 参数说明
- 发送端（ROS1 脚本）
  - `_server_host`：NX IP
  - `_server_port`：TCP 端口
  - `_scan_topic`：LaserScan 话题
- 接收端（ROS2 节点）
  - `image_height`：生成图像高度
  - `point_thickness`：距离点粗细

## 5. 注意事项
- 当前为单连接 TCP 服务，默认一个发送端接入。
- 传输内容为 JSON，每帧一行，局域网可直接使用。
- 若要提升吞吐和稳定性，可后续改为 MessagePack 或 Protobuf。
