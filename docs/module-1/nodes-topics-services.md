---
sidebar_position: 3
title: "Nodes, Topics, and Services"
description: "Hands-on guide to building ROS 2 nodes, publishing/subscribing to topics, and creating services."
keywords: [ROS 2 nodes, ROS 2 topics, ROS 2 services, publisher subscriber, rclpy]
---

# Nodes, Topics, and Services

> *"Nodes are the neurons, topics are the axons, and services are the synapses of your robot's nervous system."*

## Building a Complete Robot System

Let's build a realistic robot perception pipeline using nodes, topics, and services. We'll create a system where a camera node publishes images, an AI node processes them, and a motor node responds.

## Publisher-Subscriber Pattern

### Camera Publisher Node

```python
#!/usr/bin/env python3
"""camera_publisher.py - Publishes camera images to ROS 2"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('fps', 30)
        
        camera_idx = self.get_parameter('camera_index').value
        fps = self.get_parameter('fps').value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_idx)
        self.bridge = CvBridge()
        
        # Create publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        
        self.get_logger().info(f'📷 Camera publisher started (FPS: {fps})')
    
    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### AI Perception Subscriber

```python
#!/usr/bin/env python3
"""ai_perception.py - Subscribes to camera images and detects objects"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class AIPerceptionNode(Node):
    def __init__(self):
        super().__init__('ai_perception')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish detected objects
        self.detection_pub = self.create_publisher(
            String, '/perception/detections', 10
        )
        
        self.get_logger().info('🧠 AI Perception node started')
    
    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run object detection (simplified)
        detections = self.detect_objects(frame)
        
        # Publish detections
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)
        
        if detections:
            self.get_logger().info(
                f'Detected {len(detections)} objects: '
                f'{[d["class"] for d in detections]}'
            )
    
    def detect_objects(self, frame):
        """Simplified object detection placeholder."""
        # In practice, use YOLO, SSD, or a VLA model here
        height, width = frame.shape[:2]
        return [
            {
                "class": "cup",
                "confidence": 0.95,
                "bbox": [100, 200, 150, 250],
            }
        ]
```

## Services: Request-Response Communication

Services are for operations that need a response:

```python
#!/usr/bin/env python3
"""robot_commands.py - Service server for robot commands"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class RobotCommandServer(Node):
    def __init__(self):
        super().__init__('robot_command_server')
        
        # Create service
        self.srv = self.create_service(
            SetBool,
            '/robot/emergency_stop',
            self.emergency_stop_callback
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.is_stopped = False
        self.get_logger().info('🛑 Emergency stop service ready')
    
    def emergency_stop_callback(self, request, response):
        if request.data:
            # STOP the robot
            self.is_stopped = True
            stop_cmd = Twist()  # All zeros = stop
            self.cmd_vel_pub.publish(stop_cmd)
            
            response.success = True
            response.message = '🛑 EMERGENCY STOP ACTIVATED'
            self.get_logger().warn('EMERGENCY STOP!')
        else:
            # Resume operation
            self.is_stopped = False
            response.success = True
            response.message = '✅ Robot resumed'
            self.get_logger().info('Robot resumed normal operation')
        
        return response
```

### Calling the Service (Client)

```python
class EmergencyStopClient(Node):
    def __init__(self):
        super().__init__('emergency_stop_client')
        self.client = self.create_client(SetBool, '/robot/emergency_stop')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for emergency stop service...')
    
    def send_stop(self):
        request = SetBool.Request()
        request.data = True
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        self.get_logger().info(f'Response: {result.message}')
        return result.success
```

## Practical Exercise: Multi-Node Robot System

Build a 4-node system:

```
┌────────────┐    /image     ┌─────────────┐    /detections    ┌───────────┐
│   Camera    │─────────────▶│ AI Detector  │─────────────────▶│  Planner  │
│   Node      │              │   Node       │                  │   Node    │
└────────────┘              └─────────────┘                  └─────┬─────┘
                                                                    │ /cmd_vel
                                                              ┌─────▼─────┐
                                                              │  Motor    │
                                                              │  Control  │
                                                              └───────────┘
```

:::tip Exercise
1. Create all four nodes in separate Python files
2. Launch them together using a launch file
3. Verify communication using `ros2 topic echo`
4. Add the emergency stop service
:::

## Summary

- **Publishers** broadcast data to topics (one-to-many)
- **Subscribers** receive data from topics 
- **Services** handle request-response interactions
- **Actions** handle long-running tasks with feedback
- Design your system as a graph of loosely-coupled nodes

---

**Next:** [Building ROS 2 Packages →](/docs/module-1/building-ros2-packages)
