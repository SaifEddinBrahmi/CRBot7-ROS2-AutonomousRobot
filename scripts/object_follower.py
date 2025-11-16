#!/usr/bin/env python3
"""
Object Follower for CRBot7
Follows detected objects (e.g., person) using YOLO detections
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import math


class ObjectFollower(Node):
    """
    Follows a specific object type detected by YOLO
    Uses simple proportional control based on object position in image
    """
    
    def __init__(self):
        super().__init__('object_follower')
        
        # Parameters
        self.declare_parameter('target_class', 'person')  # What to follow
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('min_detection_confidence', 0.6)
        self.declare_parameter('target_distance', 1.5)  # meters (approximate)
        self.declare_parameter('image_center_tolerance', 50)  # pixels
        
        self.target_class = self.get_parameter('target_class').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        self.target_distance = self.get_parameter('target_distance').value
        self.center_tolerance = self.get_parameter('image_center_tolerance').value
        
        # Image dimensions (will be set from camera info)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width / 2
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.last_detection_time = self.get_clock().now()
        self.detection_timeout = 2.0  # seconds
        
        self.get_logger().info('Object Follower initialized!')
        self.get_logger().info(f'Target object: {self.target_class}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        
        # Timer for checking timeout
        self.create_timer(0.1, self.check_timeout)
    
    def detection_callback(self, msg):
        """Process YOLO detections and control robot"""
        
        # Look for target object
        target_detection = None
        max_confidence = 0.0
        
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue
            
            class_id = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # Check if this is our target class (convert class_id to name)
            # Note: class_id 0 is 'person' in COCO dataset
            if class_id == '0' and self.target_class == 'person':
                if confidence > max_confidence and confidence >= self.min_confidence:
                    target_detection = detection
                    max_confidence = confidence
        
        if target_detection is not None:
            self.follow_object(target_detection)
            self.last_detection_time = self.get_clock().now()
        else:
            # No target found - stop
            self.stop_robot()
    
    def follow_object(self, detection):
        """Generate velocity commands to follow detected object"""
        
        # Get object center position
        obj_x = detection.bbox.center.position.x
        obj_width = detection.bbox.size_x
        obj_height = detection.bbox.size_y
        
        # Calculate error from image center
        error_x = obj_x - self.image_center_x
        
        # Proportional control for angular velocity
        # Positive error = object on right = turn right (positive angular)
        angular_z = -self.angular_speed * (error_x / self.image_center_x)
        
        # Limit angular velocity
        angular_z = max(-self.angular_speed, min(self.angular_speed, angular_z))
        
        # Linear velocity based on object size (larger = closer = slower)
        # Estimate distance based on object height (rough approximation)
        # Average person height ~1.7m, at 1.5m distance should fill ~30% of frame
        estimated_distance = (self.image_height * 0.3) / (obj_height + 1e-6)
        distance_error = estimated_distance - 1.0  # normalized
        
        linear_x = self.linear_speed * max(0.0, min(1.0, distance_error))
        
        # If object is centered, move forward
        if abs(error_x) < self.center_tolerance:
            linear_x = self.linear_speed
            self.get_logger().info(f'Object centered! Moving forward at {linear_x:.2f} m/s')
        else:
            linear_x = 0.1  # Move slowly while turning
            direction = 'right' if error_x > 0 else 'left'
            self.get_logger().info(
                f'Turning {direction}: error={error_x:.0f}px, angular={angular_z:.2f} rad/s'
            )
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info(
            f'Following object: linear={linear_x:.2f}, angular={angular_z:.2f}, '
            f'obj_x={obj_x:.0f}, size={obj_width:.0f}x{obj_height:.0f}'
        )
    
    def check_timeout(self):
        """Stop robot if no detection for too long"""
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_detection > self.detection_timeout:
            self.stop_robot()
            if time_since_detection < self.detection_timeout + 1.0:  # Log only once
                self.get_logger().warn(
                    f'No {self.target_class} detected for {time_since_detection:.1f}s - Stopping'
                )
    
    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
