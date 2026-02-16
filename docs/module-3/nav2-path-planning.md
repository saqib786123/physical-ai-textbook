---
sidebar_position: 6
title: "Nav2 Path Planning"
description: "Autonomous navigation and path planning for humanoid robots using Nav2."
---

# Nav2 Path Planning

> *"Navigation is the art of getting from A to B without hitting anything — harder than it sounds for a bipedal robot."*

## Nav2 Overview

**Nav2** (Navigation 2) is the standard ROS 2 navigation framework providing autonomous navigation capabilities including path planning, obstacle avoidance, and recovery behaviors.

## Architecture

```
┌─────────────────────────────────────────────────┐
│                     Nav2                         │
├─────────────────────────────────────────────────┤
│  BT Navigator (Behavior Tree)                   │
│  ├── ComputePathToPose                          │
│  ├── FollowPath                                 │
│  ├── Spin (recovery)                            │
│  └── Wait (recovery)                            │
├─────────────────────────────────────────────────┤
│  Planner Server    │  Controller Server          │
│  ├── NavfnPlanner  │  ├── DWB Controller        │
│  ├── SmacPlanner   │  ├── MPPI Controller       │
│  └── ThetaStar     │  └── RPP Controller        │
├─────────────────────────────────────────────────┤
│  Costmap 2D                                     │
│  ├── Static Layer (from SLAM map)               │
│  ├── Obstacle Layer (real-time sensors)          │
│  └── Inflation Layer (safety margin)             │
└─────────────────────────────────────────────────┘
```

## Path Planning with Nav2

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def navigate_to_target():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()
    
    # Set goal pose
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 5.0
    goal.pose.position.y = 3.0
    goal.pose.orientation.w = 0.707
    goal.pose.orientation.z = 0.707
    
    # Navigate!
    navigator.goToPose(goal)
    
    # Monitor progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            distance = feedback.distance_remaining
            print(f'Distance remaining: {distance:.2f}m')
    
    result = navigator.getResult()
    print(f'Navigation result: {result}')
    
    rclpy.shutdown()
```

## Costmap Configuration

```yaml
# nav2_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Robot radius + safety margin
```

## Summary

Nav2 provides the complete navigation stack for autonomous robots. From global path planning to local obstacle avoidance, Nav2 enables humanoid robots to navigate complex environments safely.

---

**Next:** [Reinforcement Learning →](/docs/module-3/reinforcement-learning)
