---
sidebar_position: 5
title: "Visual SLAM"
description: "Visual Simultaneous Localization and Mapping for robot navigation."
---

# Visual SLAM (VSLAM)

> *"SLAM answers the most fundamental question a robot faces: Where am I, and what does the world look like?"*

## What is SLAM?

**SLAM** (Simultaneous Localization and Mapping) is the process of building a map of an unknown environment while simultaneously tracking the robot's location within it.

```
┌─────────────────────────────────────────┐
│              SLAM Problem                │
│                                          │
│  Given: Sensor observations over time    │
│  Find:  1. Robot's trajectory (poses)    │
│         2. Map of the environment        │
│                                          │
│  The Chicken-and-Egg:                    │
│  • Need a map to localize               │
│  • Need localization to build a map      │
│  • SLAM solves both simultaneously!      │
└─────────────────────────────────────────┘
```

## Visual SLAM vs LiDAR SLAM

| Feature | Visual SLAM | LiDAR SLAM |
|---------|------------|------------|
| **Sensor** | Camera (RGB/Depth) | LiDAR |
| **Cost** | Low ($50-350) | High ($200-10,000) |
| **Rich information** | ✅ Color, texture, objects | ❌ Geometry only |
| **Accuracy** | Good (cm-level) | Excellent (mm-level) |
| **Works in dark** | ❌ | ✅ |
| **Range** | 0.3-10m (depth camera) | 0.1-100m |

## Isaac ROS Visual SLAM

NVIDIA's `isaac_ros_visual_slam` provides GPU-accelerated Visual SLAM:

```python
# Simplified VSLAM concept
class VisualSLAM:
    def __init__(self):
        self.map = {}  # 3D point map
        self.trajectory = []  # Robot poses
        self.keyframes = []  # Important camera frames
    
    def process_frame(self, rgb_image, depth_image):
        """Process a new camera frame."""
        
        # 1. Extract features (ORB, SIFT, SuperPoint)
        features = self.extract_features(rgb_image)
        
        # 2. Match features to previous frame
        matches = self.match_features(features, self.prev_features)
        
        # 3. Estimate camera motion (essential matrix)
        transform = self.estimate_motion(matches, depth_image)
        
        # 4. Update robot pose
        new_pose = self.current_pose @ transform
        self.trajectory.append(new_pose)
        
        # 5. Triangulate new 3D points
        new_points = self.triangulate(matches, depth_image, new_pose)
        self.map.update(new_points)
        
        # 6. Loop closure detection
        if self.detect_loop_closure(features):
            self.optimize_graph()  # Bundle adjustment
        
        self.prev_features = features
        self.current_pose = new_pose
```

### Launching Isaac ROS VSLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  enable_slam_visualization:=true \
  enable_observations_view:=true \
  enable_landmarks_view:=true
```

## The SLAM Pipeline

```
Camera Frames → Feature Extraction → Feature Matching → 
    → Motion Estimation → Map Update → Loop Closure → 
    → Graph Optimization → Localized Pose + Map
```

## Occupancy Grid Maps

The output of SLAM is typically an **occupancy grid** — a 2D grid where each cell is:
- **Free** (0) — Robot can pass through
- **Occupied** (1) — Wall or obstacle
- **Unknown** (-1) — Not yet observed

```python
import numpy as np

class OccupancyGrid:
    def __init__(self, width, height, resolution=0.05):
        self.resolution = resolution  # meters per cell
        self.grid = np.full(
            (int(height/resolution), int(width/resolution)), 
            -1  # Unknown
        )
    
    def update_from_scan(self, robot_pose, laser_scan):
        """Update map from a LiDAR scan."""
        for angle, distance in laser_scan:
            # Ray-cast from robot to obstacle
            x = robot_pose[0] + distance * np.cos(angle)
            y = robot_pose[1] + distance * np.sin(angle)
            
            # Mark free cells along ray
            ray_cells = self.bresenham_line(robot_pose, (x, y))
            for cell in ray_cells[:-1]:
                self.grid[cell] = 0  # Free
            
            # Mark obstacle at end of ray
            if distance < self.max_range:
                self.grid[ray_cells[-1]] = 1  # Occupied
```

## Summary

SLAM is essential for autonomous robot navigation. Visual SLAM using Isaac ROS provides real-time, GPU-accelerated localization and mapping that enables humanoid robots to navigate unknown environments.

---

**Next:** [Nav2 Path Planning →](/docs/module-3/nav2-path-planning)
