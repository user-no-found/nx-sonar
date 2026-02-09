from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from sonar_interfaces.msg import SonarFrame


class UuvsimAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("uuvsim_adapter")

        self.declare_parameter("input_topic", "/sonar/image_raw")
        self.declare_parameter("output_topic", "/sonar/std/frame")
        self.declare_parameter("source_id", "uuvsim")
        self.declare_parameter("projection", "polar")
        self.declare_parameter("range_min_m", 0.5)
        self.declare_parameter("range_max_m", 50.0)
        self.declare_parameter("fov_rad", float(2.0 * math.pi / 3.0))
        self.declare_parameter("beam_count", 0)
        self.declare_parameter("bin_count", 0)
        self.declare_parameter("frame_id", "sonar_link")
        self.declare_parameter("image_encoding_override", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.source_id = str(self.get_parameter("source_id").value)
        self.projection = str(self.get_parameter("projection").value)
        self.range_min_m = float(self.get_parameter("range_min_m").value)
        self.range_max_m = float(self.get_parameter("range_max_m").value)
        self.fov_rad = float(self.get_parameter("fov_rad").value)
        self.beam_count = int(self.get_parameter("beam_count").value)
        self.bin_count = int(self.get_parameter("bin_count").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.image_encoding_override = str(self.get_parameter("image_encoding_override").value)

        self.publisher = self.create_publisher(SonarFrame, output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"uuvsim adapter started: {input_topic} -> {output_topic}"
        )

    @staticmethod
    def _clamp_uint16(value: int) -> int:
        return max(0, min(value, 65535))

    def _on_image(self, msg: Image) -> None:
        frame = SonarFrame()
        frame.header = msg.header
        if self.frame_id:
            frame.header.frame_id = self.frame_id

        frame.image = msg
        frame.source_id = self.source_id
        frame.projection = self.projection
        frame.range_min_m = self.range_min_m
        frame.range_max_m = self.range_max_m
        frame.fov_rad = self.fov_rad

        beam_count = self.beam_count if self.beam_count > 0 else int(msg.width)
        bin_count = self.bin_count if self.bin_count > 0 else int(msg.height)
        frame.beam_count = self._clamp_uint16(beam_count)
        frame.bin_count = self._clamp_uint16(bin_count)

        if self.image_encoding_override:
            frame.image_encoding = self.image_encoding_override
        elif msg.encoding:
            frame.image_encoding = msg.encoding
        else:
            frame.image_encoding = "mono8"

        self.publisher.publish(frame)


def main() -> None:
    rclpy.init()
    node = UuvsimAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
