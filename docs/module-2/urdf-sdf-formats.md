---
sidebar_position: 2
title: "URDF and SDF Formats"
description: "Understanding robot and world description formats: URDF vs SDF, and when to use each."
keywords: [URDF, SDF, robot description, world description, simulation formats]
---

# URDF and SDF Formats

> *"URDF describes the robot. SDF describes the world. Together, they define the entire simulation."*

## URDF vs SDF

| Feature | URDF | SDF |
|---------|------|-----|
| **Purpose** | Robot description only | Robot + World description |
| **Loops** | ❌ Tree structure only | ✅ Supports kinematic loops |
| **Sensors** | ❌ Via Gazebo extensions | ✅ Native sensor support |
| **World** | ❌ No world elements | ✅ Full world (lights, models, physics) |
| **Materials** | Basic colors | Full PBR materials |
| **Used by** | ROS 2, MoveIt | Gazebo, NVIDIA Isaac |

In practice, you'll use **URDF for your robot** (with Xacro) and **SDF for the world** — Gazebo automatically converts URDF to SDF internally.

## SDF World File Structure

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_test_world">
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Scene (rendering settings) -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.85 1.0 1</background>
      <shadows>true</shadows>
    </scene>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" 
            name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-sensors-system" 
            name="gz::sim::systems::Sensors">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" 
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Models (objects in the world) -->
    <!-- ... -->
    
  </world>
</sdf>
```

## Building a Lab Environment

```xml
<!-- A realistic lab environment for humanoid testing -->
<model name="lab_floor">
  <static>true</static>
  <link name="floor">
    <collision name="collision">
      <geometry>
        <plane><normal>0 0 1</normal><size>20 20</size></plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>    <!-- Static friction coefficient -->
            <mu2>0.6</mu2>   <!-- Dynamic friction coefficient -->
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane><normal>0 0 1</normal><size>20 20</size></plane>
      </geometry>
    </visual>
  </link>
</model>

<!-- Stairs (for bipedal locomotion testing) -->
<model name="stairs">
  <static>true</static>
  <pose>5 0 0 0 0 0</pose>
  
  <!-- Step 1 -->
  <link name="step1">
    <pose>0 0 0.1 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>1.0 0.3 0.2</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>1.0 0.3 0.2</size></box></geometry>
    </visual>
  </link>
  
  <!-- Step 2 -->
  <link name="step2">
    <pose>0 0.3 0.3 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>1.0 0.3 0.2</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>1.0 0.3 0.2</size></box></geometry>
    </visual>
  </link>
</model>
```

## URDF Best Practices for Humanoids

### 1. Use Consistent Naming

```xml
<!-- Convention: {side}_{segment}_{type} -->
<joint name="left_shoulder_pitch_joint" type="revolute">
<link name="left_upper_arm_link">
<joint name="right_knee_pitch_joint" type="revolute">
<link name="right_shin_link">
```

### 2. Set Realistic Inertial Properties

```python
# Calculate inertia for common shapes
def box_inertia(mass, x, y, z):
    """Inertia tensor for a box."""
    return {
        'ixx': (1/12) * mass * (y**2 + z**2),
        'iyy': (1/12) * mass * (x**2 + z**2),
        'izz': (1/12) * mass * (x**2 + y**2),
    }

def cylinder_inertia(mass, radius, length):
    """Inertia tensor for a cylinder."""
    return {
        'ixx': (1/12) * mass * (3*radius**2 + length**2),
        'iyy': (1/12) * mass * (3*radius**2 + length**2),
        'izz': (1/2) * mass * radius**2,
    }
```

### 3. Collision vs Visual Geometry

```xml
<!-- Use simplified collision geometry for performance -->
<link name="hand_link">
  <!-- Detailed visual mesh -->
  <visual>
    <geometry>
      <mesh>
        <uri>meshes/hand_detailed.dae</uri>
        <scale>1 1 1</scale>
      </mesh>
    </geometry>
  </visual>
  
  <!-- Simplified collision box -->
  <collision>
    <geometry>
      <box><size>0.1 0.08 0.04</size></box>
    </geometry>
  </collision>
</link>
```

## Summary

Understanding URDF and SDF is essential for robot simulation. URDF describes your robot's structure, while SDF describes the world it operates in. Together, they create complete simulation environments.

---

**Next:** [Physics Simulation →](/docs/module-2/physics-simulation)
