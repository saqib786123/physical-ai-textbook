---
sidebar_position: 2
title: "ROS 2 Architecture"
description: "Deep dive into ROS 2 architecture: DDS, QoS, executors, and the computational graph."
keywords: [ROS 2 architecture, DDS, QoS, executor, computational graph]
---

# ROS 2 Architecture

> *"Understanding the architecture of ROS 2 is like understanding the circulatory system of your robot."*

## The Computational Graph

ROS 2 organizes robot software as a **computational graph** — a network of nodes connected by communication channels:

```
                    ┌──────────────┐
                    │  ROS 2 Graph  │
                    └──────┬───────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐       ┌────▼────┐       ┌────▼────┐
   │ Camera   │       │ SLAM    │       │ Nav2    │
   │ Driver   │──────▶│ Node    │──────▶│ Planner │
   │          │ image  │         │ map   │         │
   └─────────┘       └────┬────┘       └────┬────┘
                          │                  │
                     ┌────▼────┐       ┌────▼────┐
                     │ RViz2   │       │ Motor   │
                     │ Vis     │       │ Control │
                     └─────────┘       └─────────┘
```

## DDS: The Communication Layer

ROS 2 uses **DDS (Data Distribution Service)** as its underlying communication protocol. DDS is an industry standard used in:
- Military systems
- Air traffic control
- Financial trading platforms
- Medical devices

### Why DDS?

| Feature | Benefit for Robotics |
|---------|---------------------|
| **Discovery** | Nodes find each other automatically — no master needed |
| **QoS** | Fine-grained control over reliability and performance |
| **Security** | Built-in encryption and authentication |
| **Real-time** | Deterministic communication for motor control |
| **Multi-platform** | Works across networks, containers, and VMs |

### Supported DDS Implementations

```bash
# Check which DDS is being used
echo $RMW_IMPLEMENTATION

# Common DDS implementations:
# - rmw_fastrtps_cpp (Fast-DDS by eProsima) — Default
# - rmw_cyclonedds_cpp (Cyclone DDS by Eclipse) — Often recommended
# - rmw_connextdds (RTI Connext) — Enterprise/commercial

# Switch DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Quality of Service (QoS)

QoS profiles control **how** messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# For sensor data (fast, may drop)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# For critical commands (reliable, guaranteed delivery)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# For map data (reliable, persistent)
map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

### QoS Policies Explained

| Policy | Options | When to Use |
|--------|---------|-------------|
| **Reliability** | RELIABLE / BEST_EFFORT | RELIABLE for commands; BEST_EFFORT for sensors |
| **Durability** | VOLATILE / TRANSIENT_LOCAL | TRANSIENT_LOCAL for maps, configs |
| **History** | KEEP_LAST / KEEP_ALL | KEEP_LAST with depth for most cases |
| **Deadline** | Duration | For real-time control loops |
| **Liveliness** | AUTOMATIC / MANUAL | For detecting dead nodes |

## Executors: Managing Callbacks

ROS 2 uses **executors** to manage how callbacks (timer, subscription, service) are processed:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    
    camera_node = CameraNode()
    ai_node = AIPerceptionNode()
    motor_node = MotorControlNode()
    
    # Multi-threaded executor for parallel processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(camera_node)
    executor.add_node(ai_node)
    executor.add_node(motor_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

### Executor Types

| Executor | Behavior | Use Case |
|----------|----------|----------|
| **SingleThreadedExecutor** | One callback at a time | Simple nodes |
| **MultiThreadedExecutor** | Parallel callbacks | Complex robots with multiple nodes |
| **EventsExecutor** | Event-driven (new in Humble) | Low-latency applications |

## Node Lifecycle

ROS 2 supports **managed lifecycle nodes** for deterministic startup and shutdown:

```
┌──────────┐    configure     ┌──────────┐
│           │ ──────────────▶ │           │
│ Unconfigured│               │ Inactive  │
│           │ ◀────────────── │           │
└──────────┘    cleanup       └─────┬────┘
                                     │ activate
                               ┌─────▼────┐
                               │           │
                               │  Active   │ ← Normal operation
                               │           │
                               └─────┬────┘
                                     │ deactivate
                               ┌─────▼────┐
                               │           │
                               │ Inactive  │
                               │           │
                               └──────────┘
```

```python
from rclpy.lifecycle import Node as LifecycleNode, TransitionCallbackReturn

class ManagedSensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('managed_sensor')
    
    def on_configure(self, state):
        """Initialize hardware connections."""
        self.get_logger().info('Configuring sensor...')
        self.sensor = initialize_sensor()
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        """Start publishing sensor data."""
        self.get_logger().info('Sensor active!')
        self.timer = self.create_timer(0.033, self.publish_data)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        """Stop publishing."""
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        """Release hardware resources."""
        self.sensor.close()
        return TransitionCallbackReturn.SUCCESS
```

## Message Types

ROS 2 uses strongly-typed messages defined in `.msg` files:

### Common Message Types

```python
# Geometry
from geometry_msgs.msg import (
    Twist,          # Linear + angular velocity (cmd_vel)
    Pose,           # Position + orientation
    Point,          # 3D point (x, y, z)
    Quaternion,     # Orientation as quaternion
    Transform,      # Position + orientation transform
)

# Sensors
from sensor_msgs.msg import (
    Image,          # Camera image
    LaserScan,      # 2D LiDAR scan
    PointCloud2,    # 3D point cloud
    Imu,            # IMU data (accel + gyro)
    JointState,     # Robot joint positions/velocities
)

# Navigation
from nav_msgs.msg import (
    OccupancyGrid,  # 2D map
    Odometry,       # Position + velocity
    Path,           # Planned path
)
```

### Custom Messages

```
# CustomHumanoidState.msg
# Define in your package's msg/ directory

string robot_name
float64 battery_level
geometry_msgs/Pose base_pose
float64[] joint_positions    # Array of joint angles
bool is_balanced
string current_task
```

## Summary

Understanding ROS 2's architecture is essential for designing efficient, reliable robot systems. The DDS-based communication, QoS policies, lifecycle management, and executor model give you fine-grained control over your robot's software.

### Key Takeaways

- **DDS** provides decentralized, secure, real-time communication
- **QoS** lets you trade reliability for performance as needed
- **Executors** manage parallel callback processing
- **Lifecycle nodes** enable deterministic startup/shutdown
- **Typed messages** ensure communication correctness

---

**Next:** [Nodes, Topics, and Services →](/docs/module-1/nodes-topics-services)
