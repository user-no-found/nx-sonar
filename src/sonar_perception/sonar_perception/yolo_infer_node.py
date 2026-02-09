from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sonar_interfaces.msg import SonarDetection
from sonar_interfaces.msg import SonarDetectionArray
from sonar_interfaces.msg import SonarFrame

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover
    YOLO = None  # type: ignore


class YoloInferNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_infer")

        self.declare_parameter("input_topic", "/sonar/std/frame")
        self.declare_parameter("output_topic", "/sonar/detections")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_det", 100)
        self.declare_parameter("publish_empty_when_unavailable", True)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.publish_empty_when_unavailable = bool(
            self.get_parameter("publish_empty_when_unavailable").value
        )

        self.bridge = CvBridge()
        self.model: Any = None

        self.publisher = self.create_publisher(SonarDetectionArray, output_topic, 10)
        self.subscription = self.create_subscription(
            SonarFrame,
            input_topic,
            self._on_frame,
            qos_profile_sensor_data,
        )

        self._load_model()
        self.get_logger().info(
            f"yolo infer node started: {input_topic} -> {output_topic}"
        )

    def _load_model(self) -> None:
        if YOLO is None:
            self.get_logger().warning(
                "ultralytics is not installed; node will publish empty detections."
            )
            return

        if not self.model_path:
            self.get_logger().warning(
                "model_path is empty; node will publish empty detections."
            )
            return

        try:
            self.model = YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().error(f"failed to load model: {exc}")
            self.model = None

    def _publish_empty(self, frame: SonarFrame) -> None:
        output = SonarDetectionArray()
        output.header = frame.header
        output.source_id = frame.source_id
        self.publisher.publish(output)

    @staticmethod
    def _to_model_image(cv_image: np.ndarray) -> np.ndarray:
        if cv_image.ndim == 2:
            return np.stack([cv_image, cv_image, cv_image], axis=-1)
        if cv_image.ndim == 3 and cv_image.shape[2] == 1:
            return np.repeat(cv_image, 3, axis=2)
        return cv_image

    @staticmethod
    def _class_name(names: Any, class_id: int) -> str:
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, list) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def _on_frame(self, frame: SonarFrame) -> None:
        if self.model is None:
            if self.publish_empty_when_unavailable:
                self._publish_empty(frame)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame.image, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}")
            if self.publish_empty_when_unavailable:
                self._publish_empty(frame)
            return

        model_image = self._to_model_image(np.asarray(cv_image))

        try:
            results = self.model.predict(
                source=model_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device=self.device,
                max_det=self.max_det,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"inference failed: {exc}")
            if self.publish_empty_when_unavailable:
                self._publish_empty(frame)
            return

        output = SonarDetectionArray()
        output.header = frame.header
        output.source_id = frame.source_id

        if not results:
            self.publisher.publish(output)
            return

        result = results[0]
        boxes = getattr(result, "boxes", None)
        names = getattr(result, "names", {})

        if boxes is None:
            self.publisher.publish(output)
            return

        for box in boxes:
            det = SonarDetection()

            class_id = int(box.cls[0].item()) if box.cls is not None else 0
            score = float(box.conf[0].item()) if box.conf is not None else 0.0
            xywh = box.xywh[0].tolist()

            det.class_id = class_id if class_id >= 0 else 0
            det.class_name = self._class_name(names, det.class_id)
            det.score = score
            det.x_center = float(xywh[0])
            det.y_center = float(xywh[1])
            det.width = float(xywh[2])
            det.height = float(xywh[3])

            output.detections.append(det)

        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = YoloInferNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
