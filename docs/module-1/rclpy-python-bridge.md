---
sidebar_position: 6
title: "Python Bridge with rclpy"
description: "Using rclpy to bridge Python AI agents to ROS 2 robot controllers."
keywords: [rclpy, Python ROS 2, AI integration, robot control, Python bridge]
---

# Python Bridge with rclpy

> *"rclpy bridges the world of Python AI libraries (PyTorch, TensorFlow, OpenAI) with the world of robot hardware (motors, sensors, actuators)."*

## Why Python in Robotics?

Python is the language of AI — PyTorch, TensorFlow, Hugging Face, OpenAI APIs — all are Python-first. **rclpy** (ROS Client Library for Python) connects these AI tools directly to ROS 2.

```
┌──────────────────┐     ┌──────────────────┐     ┌─────────────┐
│ Python AI World   │     │     rclpy         │     │ Robot World  │
│                   │     │   (The Bridge)    │     │              │
│ • PyTorch        │────▶│ • ROS 2 Nodes     │────▶│ • Motors     │
│ • OpenAI API     │     │ • Publishers      │     │ • Sensors    │
│ • Whisper        │     │ • Subscribers     │     │ • Cameras    │
│ • LangChain      │     │ • Services        │     │ • LiDAR      │
│ • NumPy          │     │ • Actions         │     │ • IMU        │
└──────────────────┘     └──────────────────┘     └─────────────┘
```

## Integrating PyTorch with ROS 2

```python
#!/usr/bin/env python3
"""ai_vision_node.py - PyTorch object detection in ROS 2"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import torch
import numpy as np

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision')
        
        # Load PyTorch model
        self.get_logger().info('Loading YOLOv8 model...')
        self.model = torch.hub.load(
            'ultralytics/yolov5', 'yolov5s', pretrained=True
        )
        self.model.eval()
        
        if torch.cuda.is_available():
            self.model = self.model.cuda()
            self.get_logger().info('🟢 Running on GPU')
        else:
            self.get_logger().warn('🟡 Running on CPU (slower)')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.detect_callback, 10
        )
        
        # Publish detections
        self.pub = self.create_publisher(
            Detection2DArray, '/perception/detections', 10
        )
        
        self.get_logger().info('🧠 AI Vision node ready')
    
    def detect_callback(self, msg: Image):
        # Convert ROS Image to numpy
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        
        # Run inference
        with torch.no_grad():
            results = self.model(frame)
        
        # Convert to ROS detections
        det_array = Detection2DArray()
        det_array.header = msg.header
        
        for *xyxy, conf, cls in results.xyxy[0]:
            det = Detection2D()
            det.bbox.center.position.x = float((xyxy[0] + xyxy[2]) / 2)
            det.bbox.center.position.y = float((xyxy[1] + xyxy[3]) / 2)
            det.bbox.size_x = float(xyxy[2] - xyxy[0])
            det.bbox.size_y = float(xyxy[3] - xyxy[1])
            det_array.detections.append(det)
        
        self.pub.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Integrating OpenAI with ROS 2

```python
#!/usr/bin/env python3
"""llm_planner_node.py - Use GPT for robot task planning"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        self.declare_parameter('openai_api_key', '')
        api_key = self.get_parameter('openai_api_key').value
        self.client = openai.OpenAI(api_key=api_key)
        
        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, '/voice/command',
            self.plan_callback, 10
        )
        
        # Publish action plan
        self.plan_pub = self.create_publisher(
            String, '/planner/action_sequence', 10
        )
        
        self.system_prompt = """You are a robot task planner. 
        Given a natural language command, output a JSON array of 
        ROS 2 actions the robot should execute.
        Available actions: navigate_to, pick_up, place_on, 
        look_at, say, wait, open_gripper, close_gripper.
        Each action has parameters specific to its type."""
        
        self.get_logger().info('🤖 LLM Planner ready')
    
    def plan_callback(self, msg: String):
        command = msg.data
        self.get_logger().info(f'Planning for: "{command}"')
        
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            response_format={"type": "json_object"},
            temperature=0.1,
        )
        
        plan = response.choices[0].message.content
        
        plan_msg = String()
        plan_msg.data = plan
        self.plan_pub.publish(plan_msg)
        
        self.get_logger().info(f'Published plan: {plan}')
```

## Async Patterns in rclpy

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class AsyncRobotNode(Node):
    """Node that handles multiple async operations."""
    
    def __init__(self):
        super().__init__('async_robot')
        
        # Reentrant callback group allows parallel execution
        self.cb_group = ReentrantCallbackGroup()
        
        # Fast sensor loop (100 Hz)
        self.sensor_timer = self.create_timer(
            0.01, self.sensor_callback,
            callback_group=self.cb_group
        )
        
        # Slow AI processing (5 Hz)
        self.ai_timer = self.create_timer(
            0.2, self.ai_callback,
            callback_group=self.cb_group
        )
    
    def sensor_callback(self):
        """High-frequency sensor processing."""
        # Must be fast! No AI inference here.
        pass
    
    def ai_callback(self):
        """Lower-frequency AI processing."""
        # Can be slower — runs in parallel with sensor_callback
        pass

def main():
    rclpy.init()
    node = AsyncRobotNode()
    
    # Use multi-threaded executor for parallel callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

## Summary

rclpy is the critical bridge that connects the Python AI ecosystem to ROS 2 robot systems. Mastering it enables you to deploy any AI model — from PyTorch vision models to OpenAI language models — directly on a robot.

---

**Next:** [URDF: Humanoid Robot Description →](/docs/module-1/urdf-humanoid-description)
