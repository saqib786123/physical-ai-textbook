---
sidebar_position: 1
title: "Introduction to ROS 2"
description: "Getting started with ROS 2 (Robot Operating System 2) — the middleware backbone of modern robotics."
keywords: [ROS 2, Robot Operating System, robotics middleware, ROS 2 Humble, rclpy]
---

# Introduction to ROS 2

> *"ROS 2 is to robotics what an operating system is to a computer — the essential middleware that makes everything work together."*

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is not actually an operating system — it's a **middleware framework** for robot software development. It provides:

- **Communication infrastructure** — Way to pass messages between different parts of a robot system
- **Hardware abstraction** — Standard interfaces for sensors and actuators
- **Package ecosystem** — Thousands of reusable robotics packages
- **Tooling** — Visualization (RViz2), logging, debugging, and simulation integration

### Why "ROS 2" (Not ROS 1)?

ROS 1 was groundbreaking but had critical limitations:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-time** | ❌ Not real-time capable | ✅ Real-time support (DDS) |
| **Security** | ❌ No built-in security | ✅ DDS Security (encryption, auth) |
| **Multi-robot** | ❌ Single master | ✅ Decentralized (no master) |
| **Reliability** | ❌ TCP only, unreliable | ✅ QoS policies (reliable/best-effort) |
| **Platform** | ❌ Linux only | ✅ Linux, Windows, macOS |
| **Lifecycle** | ❌ No node lifecycle | ✅ Managed node lifecycle |

For this course, we use **ROS 2 Humble Hawksbill** (LTS release) or **ROS 2 Iron Irwini**.

## Core Architecture

ROS 2 uses a **distributed** architecture based on DDS (Data Distribution Service):

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Node A     │     │   Node B     │     │   Node C     │
│ (Camera)     │     │ (AI Model)   │     │ (Motor)      │
│              │     │              │     │              │
│  Publisher ──┼────▶│  Subscriber  │     │              │
│              │     │  Publisher ──┼────▶│  Subscriber  │
│              │     │              │     │              │
└─────────────┘     └─────────────┘     └─────────────┘
        │                   │                   │
        └───────────────────┼───────────────────┘
                            │
                    ┌───────────────┐
                    │   DDS Layer    │
                    │ (Discovery,    │
                    │  Transport)    │
                    └───────────────┘
```

**No central master!** Nodes discover each other automatically through the DDS discovery protocol.

## Key Concepts

### 1. Nodes

A **node** is a single, modular process that performs a specific function:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('Hello from Physical AI! 🤖')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Topics

**Topics** are named buses for streaming data. Nodes **publish** to topics and **subscribe** to topics:

```python
from std_msgs.msg import String
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Publish camera images to the '/camera/image' topic
        self.publisher = self.create_publisher(
            Image,              # Message type
            '/camera/image',    # Topic name
            10                  # Queue size
        )
        
        # Publish at 30 FPS
        self.timer = self.create_timer(1/30, self.publish_frame)
    
    def publish_frame(self):
        msg = Image()
        # ... fill in image data ...
        self.publisher.publish(msg)
        self.get_logger().debug('Published camera frame')
```

### 3. Services

**Services** are request-response interactions (like HTTP):

```python
from example_interfaces.srv import AddTwoInts

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
    
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### 4. Actions

**Actions** are for long-running tasks with feedback (like "navigate to position"):

```python
# Actions provide:
# 1. Goal - What to achieve
# 2. Feedback - Progress updates
# 3. Result - Final outcome

# Example: Navigate to a point
# Goal: target_position = (3.0, 5.0)
# Feedback: current_position = (1.5, 2.3), distance_remaining = 3.2m
# Result: success = True, final_position = (3.0, 5.0)
```

## Installation (Ubuntu 22.04)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source the setup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

## Your First ROS 2 Program

```python
#!/usr/bin/env python3
"""
physical_ai_hello.py
Your first Physical AI ROS 2 node!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAINode(Node):
    """A simple node that publishes Physical AI messages."""
    
    def __init__(self):
        super().__init__('physical_ai_node')
        
        # Create a publisher
        self.publisher = self.create_publisher(
            String, 
            '/physical_ai/status', 
            10
        )
        
        # Create a timer (publish every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info('🤖 Physical AI Node initialized!')
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Physical AI Status #{self.count}: All systems nominal'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## ROS 2 Command Line Tools

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# See what's being published on a topic
ros2 topic echo /physical_ai/status

# Get info about a topic
ros2 topic info /physical_ai/status

# Publish from command line
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"

# List all services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \
  "{a: 5, b: 3}"
```

## The ROS 2 Ecosystem in Physical AI

```
┌─────────────────────────────────────────────────────┐
│              Physical AI with ROS 2                  │
│                                                      │
│  Perception:    image_pipeline, pointcloud_to_laserscan
│  Navigation:    nav2, slam_toolbox, robot_localization
│  Manipulation:  MoveIt2, ros2_control               │
│  Simulation:    gazebo_ros2, isaac_ros               │
│  Visualization: rviz2, foxglove                      │
│  AI/ML:         rclpy + PyTorch/TensorFlow           │
│  Voice:         whisper_ros (custom)                  │
│  Planning:      behavior_tree, plansys2              │
└─────────────────────────────────────────────────────┘
```

## Summary

ROS 2 is the **backbone** of modern robotics software. It provides the communication infrastructure, tooling, and ecosystem that makes building complex multi-component robot systems practical.

### Key Takeaways

- ROS 2 is **middleware**, not an operating system
- Communication is **decentralized** (no master node)
- **Topics** for streaming data, **Services** for request-response, **Actions** for long-running tasks
- ROS 2 Humble is the recommended LTS release
- Use `rclpy` for Python-based development

---

**Next:** [ROS 2 Architecture Deep Dive →](/docs/module-1/ros2-architecture)
