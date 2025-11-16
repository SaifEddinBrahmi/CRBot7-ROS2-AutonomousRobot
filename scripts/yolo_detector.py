#!/usr/bin/env python3
"""
YOLO Object Detector for CRBot7
Detects objects in real-time from robot camera feed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Run: pip install ultralytics")


class YOLODetector(Node):
    """
    ROS2 Node for YOLO-based object detection
    Subscribes to camera feed and publishes detection results
    """
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_name', 'yolov8n.pt')  # nano model (fastest)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/robot_cam')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        
        # Get parameters
        model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        device = self.get_parameter('device').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLO model
        if YOLO_AVAILABLE:
            try:
                self.get_logger().info(f'Loading YOLO model: {model_name}...')
                self.model = YOLO(model_name)
                self.model.to(device)
                self.get_logger().info(f'YOLO model loaded successfully on {device}!')
                self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.model = None
        else:
            self.get_logger().error('Ultralytics not available. Install with: pip install ultralytics')
            self.model = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/yolo/annotated_image',
                10
            )
        
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info('YOLO Detector Node initialized!')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info('Publishing to: /yolo/detections, /yolo/annotated_image')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        if self.model is None:
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            
            # Process results
            self.process_detections(results, msg.header)
            
            # Publish annotated image if enabled
            if self.publish_annotated and len(results) > 0:
                annotated_image = results[0].plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_detections(self, results, header):
        """Convert YOLO results to ROS Detection2DArray message"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        if len(results) == 0:
            self.detections_pub.publish(detection_array)
            return
        
        result = results[0]
        boxes = result.boxes
        
        if boxes is None or len(boxes) == 0:
            self.detections_pub.publish(detection_array)
            return
        
        for box in boxes:
            detection = Detection2D()
            detection.header = header
            
            # Bounding box (x_center, y_center, width, height)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
            
            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box.cls[0]))
            hypothesis.hypothesis.score = float(box.conf[0])
            
            detection.results.append(hypothesis)
            detection_array.detections.append(detection)
            
            # Log detection
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            confidence = float(box.conf[0])
            
            self.get_logger().info(
                f'Detected: {class_name} ({confidence:.2f}) at '
                f'[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]'
            )
            self.detection_count += 1
        
        # Publish detections
        self.detections_pub.publish(detection_array)
        
        # Log statistics every 100 frames
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Statistics: {self.frame_count} frames, '
                f'{self.detection_count} total detections'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
