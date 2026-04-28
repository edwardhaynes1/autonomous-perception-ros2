#!/usr/bin/env python3
import time
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from perception_interfaces.msg import Detection2D, Detection2DArray

COCO_PALETTE = np.random.default_rng(42).integers(100, 255, size=(80, 3), dtype=np.uint8)

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.40)
        self.declare_parameter("input_topic", "/camera/color/image")
        self.declare_parameter("output_topic", "/perception/detections")
        self.declare_parameter("annotated_topic", "/perception/annotated_image")
        self.declare_parameter("device", "cpu")

        model_name = self.get_parameter("model_name").value
        self.conf  = self.get_parameter("confidence_threshold").value
        device     = self.get_parameter("device").value

        self.get_logger().info(f"Loading {model_name} on {device}")
        from ultralytics import YOLO
        self.model = YOLO(model_name)
        self.model.to(device)

        self.bridge = CvBridge()
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub = self.create_subscription(
            Image, self.get_parameter("input_topic").value, self.cb, be)
        self.pub_det = self.create_publisher(
            Detection2DArray, self.get_parameter("output_topic").value, rel)
        self.pub_img = self.create_publisher(
            Image, self.get_parameter("annotated_topic").value, rel)

        self._fps_ema = 0.0
        self._last_ts = None
        self._lat_win = deque(maxlen=30)
        self.get_logger().info("YoloDetectorNode ready.")

    def cb(self, msg: Image):
        t0 = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(str(e)); return

        h, w = frame.shape[:2]
        t1 = time.perf_counter()
        results = self.model(frame, conf=self.conf, verbose=False)
        infer_ms = (time.perf_counter() - t1) * 1000.0

        arr = Detection2DArray()
        arr.header = msg.header
        arr.image_width = w
        arr.image_height = h
        annotated = frame.copy()

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cid  = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                name = self.model.names.get(cid, f"cls_{cid}")
                d = Detection2D(header=msg.header, class_name=name, class_id=cid,
                                confidence=conf, x_min=x1, y_min=y1, x_max=x2, y_max=y2,
                                cx=float((x1+x2)/2), cy=float((y1+y2)/2))
                arr.detections.append(d)
                col = tuple(int(c) for c in COCO_PALETTE[cid % 80])
                cv2.rectangle(annotated, (x1,y1), (x2,y2), col, 2)
                label = f"{name} {conf:.2f}"
                (tw,th),_ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(annotated, (x1, y1-th-6), (x1+tw+4, y1), col, -1)
                cv2.putText(annotated, label, (x1+2, y1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)

        total_ms = (time.perf_counter() - t0) * 1000.0
        now = time.perf_counter()
        if self._last_ts:
            fps = 1.0 / max(now - self._last_ts, 1e-6)
            self._fps_ema = 0.1 * fps + 0.9 * self._fps_ema
        self._last_ts = now
        self._lat_win.append(infer_ms)

        arr.inference_time_ms = infer_ms
        arr.total_time_ms = total_ms
        arr.fps = self._fps_ema

        hud = (f"Model:{self.get_parameter('model_name').value} "
               f"FPS:{self._fps_ema:.1f} Infer:{infer_ms:.1f}ms Dets:{len(arr.detections)}")
        cv2.putText(annotated, hud, (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(annotated, hud, (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,255,100), 1, cv2.LINE_AA)

        self.pub_det.publish(arr)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        if len(self._lat_win) == 30:
            a = np.array(self._lat_win)
            self.get_logger().info(
                f"[30-frame] mean={a.mean():.1f}ms p95={np.percentile(a,95):.1f}ms FPS={self._fps_ema:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
