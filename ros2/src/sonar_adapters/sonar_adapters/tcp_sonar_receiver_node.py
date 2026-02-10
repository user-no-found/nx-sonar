from __future__ import annotations

import json
import math
import queue
import socket
import threading
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class TcpSonarReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("tcp_sonar_receiver")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 29001)
        self.declare_parameter("output_topic", "/sonar/image_raw")
        self.declare_parameter("frame_id", "sonar_link")
        self.declare_parameter("image_height", 256)
        self.declare_parameter("point_thickness", 1)

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.point_thickness = int(self.get_parameter("point_thickness").value)

        self.publisher = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        self.packet_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=100)
        self.timer = self.create_timer(0.01, self._drain_queue_and_publish)

        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"TCP 声呐接收器已启动: {self.bind_host}:{self.bind_port} -> {output_topic}"
        )

    def _run_server(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.bind_host, self.bind_port))
        server.listen(1)

        while rclpy.ok():
            self.get_logger().info("等待 TCP 客户端连接...")
            try:
                client, addr = server.accept()
            except OSError as exc:
                self.get_logger().error(f"接收连接失败: {exc}")
                continue

            self.get_logger().info(f"客户端已连接: {addr[0]}:{addr[1]}")
            client.settimeout(1.0)
            buf = b""

            while rclpy.ok():
                try:
                    data = client.recv(8192)
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.get_logger().warning(f"读取连接失败: {exc}")
                    break

                if not data:
                    self.get_logger().warning("客户端已断开。")
                    break

                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line:
                        continue
                    try:
                        packet = json.loads(line.decode("utf-8"))
                    except Exception as exc:
                        self.get_logger().warning(f"JSON 解析失败，已丢弃一帧: {exc}")
                        continue

                    if self.packet_queue.full():
                        try:
                            _ = self.packet_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.packet_queue.put(packet)

            try:
                client.close()
            except OSError:
                pass

    def _drain_queue_and_publish(self) -> None:
        newest_packet: dict[str, Any] | None = None
        while True:
            try:
                newest_packet = self.packet_queue.get_nowait()
            except queue.Empty:
                break

        if newest_packet is None:
            return

        msg = self._packet_to_image_msg(newest_packet)
        if msg is not None:
            self.publisher.publish(msg)

    def _packet_to_image_msg(self, packet: dict[str, Any]) -> Image | None:
        ranges = packet.get("ranges", [])
        if not isinstance(ranges, list) or len(ranges) == 0:
            return None

        width = len(ranges)
        height = max(16, self.image_height)
        image = np.zeros((height, width), dtype=np.uint8)

        range_min = float(packet.get("range_min", 0.1))
        range_max = float(packet.get("range_max", 50.0))
        span = max(1e-6, range_max - range_min)
        thickness = max(0, self.point_thickness)

        for col, value in enumerate(ranges):
            if value is None:
                continue
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if (not math.isfinite(distance)) or distance < range_min or distance > range_max:
                continue

            normalized = (distance - range_min) / span
            row = height - 1 - int(normalized * (height - 1))
            row = max(0, min(height - 1, row))

            lo = max(0, row - thickness)
            hi = min(height, row + thickness + 1)
            image[lo:hi, col] = 255

        msg = Image()
        msg.header.frame_id = str(packet.get("frame_id", self.frame_id)) or self.frame_id

        stamp_sec = packet.get("stamp_sec", None)
        stamp_nsec = packet.get("stamp_nsec", None)
        if isinstance(stamp_sec, int) and isinstance(stamp_nsec, int):
            msg.header.stamp.sec = stamp_sec
            msg.header.stamp.nanosec = stamp_nsec
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        msg.height = height
        msg.width = width
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = width
        msg.data = image.tobytes()
        return msg


def main() -> None:
    rclpy.init()
    node = TcpSonarReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
