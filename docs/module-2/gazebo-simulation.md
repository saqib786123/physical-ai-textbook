---
sidebar_position: 1
title: "Gazebo Simulation Environment"
description: "Setting up and using Gazebo for robot simulation with ROS 2 integration."
keywords: [Gazebo, robot simulation, physics simulation, ROS 2 Gazebo, Ignition Gazebo]
---

# Gazebo Simulation Environment

> *"In simulation, you can crash a million-dollar robot a thousand times and learn from every failure — for free."*

## Why Simulate?

Simulation is the cornerstone of Physical AI development:

| Real Hardware | Simulation |
|--------------|------------|
| Expensive ($16K+ for a humanoid) | Free |
| Slow iteration (physical setup time) | Instant reset |
| Dangerous (robot can break, injure) | Perfectly safe |
| Limited data (one run at a time) | Parallel environments |
| Weather/lighting dependent | Fully controllable |

## Gazebo Overview

**Gazebo** (now called **Gazebo Sim**, formerly Ignition Gazebo) is the most widely used open-source robotics simulator. It provides:

- **Physics Engine** — Simulate gravity, friction, collisions, constraints
- **Sensor Simulation** — Virtual cameras, LiDAR, IMU, GPS
- **World Building** — Create environments with static/dynamic objects
- **ROS 2 Integration** — Direct communication with ROS 2 nodes
- **Plugin System** — Extend functionality with custom plugins

### Gazebo Versions

| Version | Status | ROS 2 Support |
|---------|--------|---------------|
| Gazebo Classic (11) | Legacy | ROS 2 Humble ✅ |
| Gazebo Fortress | LTS | ROS 2 Humble ✅ |
| Gazebo Garden | Stable | ROS 2 Iron ✅ |
| Gazebo Harmonic | Latest LTS | ROS 2 Jazzy ✅ |

For this course, we use **Gazebo Fortress** or **Harmonic**.

## Installation

```bash
# Install Gazebo Harmonic (Ubuntu 22.04)
sudo apt-get update
sudo apt-get install gz-harmonic

# Install ROS 2 - Gazebo bridge
sudo apt-get install ros-humble-ros-gz

# Verify installation
gz sim --version
```

## Your First Simulation

### Creating a World File

```xml
<!-- worlds/physical_ai_lab.sdf -->
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="physical_ai_lab">
    
    <!-- Physics Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- A Table -->
    <model name="table">
      <static>true</static>
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1.2 0.6 0.75</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.2 0.6 0.75</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- A Cup (graspable object) -->
    <model name="red_cup">
      <pose>2 0 0.8 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.05</mass>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.035</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.035</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.1 0.1 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

### Launching the Simulation

```bash
# Launch Gazebo with your world
gz sim worlds/physical_ai_lab.sdf

# Launch with ROS 2 bridge
ros2 launch ros_gz_sim gz_sim.launch.py \
  world_sdf_file:=worlds/physical_ai_lab.sdf
```

## Spawning a Robot

```python
# launch/spawn_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to URDF
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH', ''),
        'share', 'humanoid_description', 'urdf', 'humanoid.urdf'
    )
    
    # Read URDF content
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            'ros_gz_sim/gz_sim.launch.py',
            launch_arguments={
                'gz_args': 'worlds/physical_ai_lab.sdf'
            }.items()
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'humanoid',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1.0',
            ],
            output='screen',
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
    ])
```

## Physics Configuration

### Key Physics Parameters

```xml
<physics type="ode">
  <!-- Simulation step size (smaller = more accurate, slower) -->
  <max_step_size>0.001</max_step_size>  <!-- 1ms steps -->
  
  <!-- Real-time factor (1.0 = real-time, 0 = max speed) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- ODE solver parameters -->
  <ode>
    <solver>
      <iters>50</iters>           <!-- Solver iterations -->
      <sor>1.3</sor>              <!-- Successive over-relaxation -->
    </solver>
    <constraints>
      <cfm>0.000001</cfm>        <!-- Constraint force mixing -->
      <erp>0.2</erp>              <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Physics Engines Comparison

| Engine | Speed | Accuracy | Best For |
|--------|-------|----------|----------|
| **ODE** | Fast | Good | General robotics |
| **Bullet** | Fast | Good | Game-like physics |
| **DART** | Medium | High | Articulated bodies |
| **Simbody** | Slow | Very High | Research |

## Gazebo-ROS 2 Bridge

The bridge connects Gazebo topics to ROS 2 topics:

```bash
# Bridge a Gazebo camera to ROS 2
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@gz.msgs.Image

# Bridge velocity commands
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Bridge multiple topics
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

## Summary

Gazebo is your primary development environment for Physical AI. Master simulation first, then deploy to real hardware with confidence.

### Key Takeaways

- Simulation is **essential** — test before deploying
- Gazebo provides physics, sensors, and ROS 2 integration
- World files (SDF) define environments
- URDF/SDF define robots
- The ROS 2-Gazebo bridge connects everything

---

**Next:** [URDF and SDF Formats →](/docs/module-2/urdf-sdf-formats)
