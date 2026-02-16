---
sidebar_position: 4
title: "Sensor Systems for Robotics"
description: "Understanding the sensor systems used in humanoid robotics: cameras, LiDAR, IMUs, and force/torque sensors."
keywords: [sensors, LiDAR, IMU, cameras, depth sensors, force torque, robotics sensors]
---

# Sensor Systems for Robotics

> *"Sensors are the robot's window to the world. Without them, even the most powerful AI is blind."*

## Overview

Just as humans perceive the world through five senses, robots use **sensor systems** to perceive their environment. The quality and type of sensors fundamentally determine what a robot can perceive and how well it can act.

## Vision Systems

### RGB Cameras

The most basic sensor — a standard color camera capturing 2D images.

**Specifications to Consider:**
- **Resolution:** 1080p minimum, 4K for detailed manipulation
- **Frame Rate:** 30 FPS for navigation, 60+ FPS for manipulation
- **Field of View:** 60-120° horizontal
- **Interface:** USB3, MIPI CSI-2

```python
# Reading from a camera with OpenCV (common in ROS 2)
import cv2

cap = cv2.VideoCapture(0)  # Camera index 0

while True:
    ret, frame = cap.read()  # Read a frame
    if not ret:
        break
    
    # Frame is a numpy array of shape (height, width, 3)
    # Channels are in BGR format (OpenCV convention)
    height, width, channels = frame.shape
    print(f"Image: {width}x{height}, {channels} channels")
    
    cv2.imshow('Robot Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
```

### Depth Cameras

Depth cameras provide **distance information** for every pixel, creating a 2.5D view of the world.

**Technologies:**
| Technology | How It Works | Range | Accuracy | Example |
|------------|-------------|-------|----------|---------|
| **Structured Light** | Projects IR pattern, measures distortion | 0.3-3m | ±1mm | Intel RealSense D435i |
| **Time-of-Flight (ToF)** | Measures light round-trip time | 0.5-10m | ±10mm | Microsoft Azure Kinect |
| **Stereo Vision** | Two cameras compute disparity | 0.5-20m | Variable | ZED 2 |

**The Intel RealSense D435i** — recommended for this course:

```python
import pyrealsense2 as rs
import numpy as np

# Configure the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Get distance at center pixel
        center_distance = depth_frame.get_distance(320, 240)
        print(f"Center distance: {center_distance:.3f} meters")
        
finally:
    pipeline.stop()
```

## LiDAR (Light Detection and Ranging)

LiDAR creates a **3D point cloud** of the environment by shooting laser beams and measuring their return time.

### Types of LiDAR

| Type | Description | Use Case | Cost |
|------|-------------|----------|------|
| **2D LiDAR** | Single scanning plane | Navigation, obstacle avoidance | $100-500 |
| **3D LiDAR** | Multiple scanning planes | Full 3D mapping | $500-10,000 |
| **Solid-State** | No moving parts | Compact systems | $200-2,000 |

### Point Cloud Data

```python
# Processing LiDAR point cloud data
import numpy as np

# A point cloud is simply an array of (x, y, z) coordinates
# Each point represents where a laser beam hit a surface
point_cloud = np.array([
    [1.5, 0.3, 0.0],   # Object at 1.5m forward, 0.3m right
    [2.0, -0.1, 0.5],  # Object at 2.0m forward, 0.1m left, 0.5m up
    [0.8, 0.0, 0.0],   # Obstacle at 0.8m directly ahead!
    # ... thousands more points
])

# Filter points that are too close (potential obstacles)
min_safe_distance = 0.5  # meters
close_points = point_cloud[
    np.linalg.norm(point_cloud[:, :2], axis=1) < min_safe_distance
]

if len(close_points) > 0:
    print(f"⚠️ WARNING: {len(close_points)} points within {min_safe_distance}m!")
    print("Obstacle avoidance required!")
```

## Inertial Measurement Units (IMUs)

The IMU is the robot's **inner ear** — it measures acceleration and rotation to track body orientation and movement.

### Components

- **Accelerometer:** Measures linear acceleration (3 axes: x, y, z)
- **Gyroscope:** Measures angular velocity (3 axes: roll, pitch, yaw)
- **Magnetometer:** Measures magnetic field (compass heading)

### 6-DOF vs 9-DOF IMUs

- **6-DOF:** Accelerometer + Gyroscope (minimum for balance)
- **9-DOF:** Accelerometer + Gyroscope + Magnetometer (full orientation)

```python
# Reading IMU data (conceptual - BNO055 sensor)
class IMUReader:
    def __init__(self):
        self.gravity = 9.81  # m/s²
    
    def read_orientation(self):
        """
        Returns the robot's orientation as Euler angles:
        - Roll: Rotation around forward axis (tilting sideways)
        - Pitch: Rotation around side axis (tilting forward/backward)
        - Yaw: Rotation around vertical axis (turning left/right)
        """
        return {
            'roll': self.sensor.euler[0],    # degrees
            'pitch': self.sensor.euler[1],   # degrees
            'yaw': self.sensor.euler[2],     # degrees
        }
    
    def is_falling(self):
        """
        Detect if the robot is falling by checking
        if measured acceleration differs significantly
        from gravity (free-fall = 0g).
        """
        accel = self.sensor.linear_acceleration
        total_accel = (accel[0]**2 + accel[1]**2 + accel[2]**2) ** 0.5
        
        if total_accel < 2.0:  # Much less than 9.81 m/s²
            return True  # Free fall detected!
        return False
```

:::warning IMU Drift
All IMUs suffer from **drift** — small errors that accumulate over time. After minutes of operation, the estimated position can be meters off. This is why IMU data must be **fused** with other sensors (cameras, LiDAR) using Kalman filters or factor graphs.
:::

## Force and Torque Sensors

These sensors measure the physical forces acting on the robot, essential for:

- **Safe human interaction** — Detecting contact and limiting forces
- **Manipulation** — Knowing how hard to grip an object
- **Locomotion** — Detecting ground contact during walking

### Types

| Sensor Type | Measures | Location | Use |
|-------------|----------|----------|-----|
| **6-axis F/T** | Forces (Fx, Fy, Fz) + Torques (Tx, Ty, Tz) | Wrist, ankles | Manipulation, balance |
| **Tactile arrays** | Pressure distribution | Fingertips, palms | Grasping |
| **Joint torque** | Torque at each joint | Motor/joint | Compliance control |
| **Foot pressure** | Ground reaction forces | Feet soles | Walking stability |

## Sensor Fusion

No single sensor provides a complete picture. **Sensor fusion** combines data from multiple sensors to create a robust understanding of the environment.

### Common Fusion Approaches

```
┌─────────────────────────────────────────────────────┐
│                 SENSOR FUSION                        │
│                                                      │
│  Camera ──┐                                         │
│  Depth  ──┼──→ [Kalman Filter / EKF] ──→ State     │
│  LiDAR  ──┤              or              Estimate   │
│  IMU    ──┤    [Factor Graph / GTSAM]               │
│  F/T    ──┘                                         │
│                                                      │
│  State = Position + Orientation + Velocity +         │
│          Map + Object Locations                      │
└─────────────────────────────────────────────────────┘
```

## Sensor Setup for This Course

### Minimum Setup (Simulation Only)
- No physical sensors needed
- All sensors simulated in Gazebo / Isaac Sim

### Recommended Physical Setup
| Component | Model | Purpose |
|-----------|-------|---------|
| Depth Camera | Intel RealSense D435i | RGB + Depth + IMU |
| 2D LiDAR | RPLiDAR A1 | Navigation testing |
| Microphone | ReSpeaker USB Mic Array v2.0 | Voice commands |
| Edge Compute | NVIDIA Jetson Orin Nano | On-device inference |

## Summary

Sensors are the foundation of Physical AI perception. Understanding their capabilities, limitations, and how to combine them is essential for building robust robotic systems.

### Key Takeaways

- **Vision** (RGB + Depth) provides rich semantic understanding
- **LiDAR** provides precise 3D geometry for navigation
- **IMUs** track body orientation and detect instability
- **Force/Torque** sensors enable safe physical interaction
- **Sensor fusion** is essential — no single sensor is sufficient

---

**Next:** [Hardware Requirements →](/docs/foundations/hardware-requirements)
