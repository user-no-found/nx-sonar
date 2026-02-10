#!/usr/bin/env python3

import json
import math
import socket
import time

import rospy
from sensor_msgs.msg import LaserScan


class LaserScanTcpSender:
    def __init__(self) -> None:
        self.server_host = rospy.get_param("~server_host", "127.0.0.1")
        self.server_port = int(rospy.get_param("~server_port", 29001))
        self.scan_topic = rospy.get_param("~scan_topic", "/rexrov/sonar")
        self.connect_retry_sec = float(rospy.get_param("~connect_retry_sec", 1.0))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 3.0))

        self.sock = None
        self._connect()
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)
        rospy.loginfo(
            "激光扫描 TCP 发送器已启动: %s -> %s:%d",
            self.scan_topic,
            self.server_host,
            self.server_port,
        )

    def _connect(self) -> None:
        while not rospy.is_shutdown():
            try:
                if self.sock is not None:
                    self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.socket_timeout_sec)
                self.sock.connect((self.server_host, self.server_port))
                rospy.loginfo("已连接到网关接收端: %s:%d", self.server_host, self.server_port)
                return
            except OSError as exc:
                rospy.logwarn("连接网关失败: %s，%.1f 秒后重试", exc, self.connect_retry_sec)
                time.sleep(self.connect_retry_sec)

    @staticmethod
    def _sanitize_ranges(ranges):
        result = []
        for value in ranges:
            if value is None:
                result.append(None)
                continue
            if not math.isfinite(value):
                result.append(None)
                continue
            result.append(float(value))
        return result

    def _on_scan(self, msg: LaserScan) -> None:
        packet = {
            "type": "laser_scan_v1",
            "stamp_sec": int(msg.header.stamp.secs),
            "stamp_nsec": int(msg.header.stamp.nsecs),
            "frame_id": str(msg.header.frame_id),
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": self._sanitize_ranges(msg.ranges),
        }

        payload = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")

        try:
            if self.sock is None:
                self._connect()
            self.sock.sendall(payload)
        except OSError as exc:
            rospy.logwarn("发送失败: %s，开始重连", exc)
            self._connect()


def main() -> None:
    rospy.init_node("laser_scan_tcp_sender")
    _ = LaserScanTcpSender()
    rospy.spin()


if __name__ == "__main__":
    main()
