---
sidebar_position: 7
title: "URDF: Humanoid Robot Description"
description: "Understanding URDF (Unified Robot Description Format) for defining humanoid robot models."
keywords: [URDF, robot description, humanoid model, joints, links, Xacro]
---

# URDF: Humanoid Robot Description

> *"URDF is the blueprint of your robot — defining every link, joint, and sensor in a machine-readable format."*

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML-based format that describes a robot's physical structure:

- **Links** — Rigid body parts (torso, arms, legs, head)
- **Joints** — Connections between links (revolute, prismatic, fixed)
- **Visual** — What the robot looks like (meshes, colors)
- **Collision** — Simplified shapes for physics simulation
- **Inertial** — Mass and inertia properties for dynamics

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" 
               iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>
  
  <!-- Head -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="silver">
        <color rgba="0.8 0.8 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1.0"/>
  </joint>
  
</robot>
```

## Joint Types for Humanoids

| Joint Type | DOF | Description | Example |
|------------|-----|-------------|---------|
| **revolute** | 1 | Rotation around axis | Elbow, knee |
| **continuous** | 1 | Continuous rotation | Wheel |
| **prismatic** | 1 | Linear sliding | Telescoping arm |
| **fixed** | 0 | No movement | Sensor mount |
| **floating** | 6 | Free in space | Base link (humanoid) |
| **planar** | 3 | Movement in a plane | Rare in humanoids |

## Humanoid URDF: Joint Layout

```
                    ┌──────────┐
                    │   Head    │
                    │ (2 DOF)  │
                    └────┬─────┘
                         │ neck_pan, neck_tilt
                    ┌────┴─────┐
         ┌──────────┤  Torso   ├──────────┐
         │          │ (3 DOF)  │          │
    shoulder_l      └────┬─────┘     shoulder_r
    (3 DOF)              │           (3 DOF)
    ┌────┴────┐    waist (3 DOF)    ┌────┴────┐
    │ Upper   │     ┌────┴────┐     │ Upper   │
    │ Arm L   │     │  Pelvis │     │ Arm R   │
    └────┬────┘     └────┬────┘     └────┬────┘
    elbow_l         ┌────┴────┐     elbow_r
    (1 DOF)    hip_l│         │hip_r (1 DOF)
    ┌────┴────┐(3 DOF)     (3 DOF)  ┌────┴────┐
    │ Forearm │┌────┴──┐ ┌──┴────┐  │ Forearm │
    │   L     ││Thigh L│ │Thigh R│  │   R     │
    └────┬────┘└───┬───┘ └───┬───┘  └────┬────┘
    wrist_l   knee_l     knee_r     wrist_r
    (2 DOF)   (1 DOF)    (1 DOF)   (2 DOF)
    ┌────┴──┐ ┌──┴───┐   ┌───┴──┐  ┌──┴────┐
    │ Hand L│ │Shin L │   │Shin R│  │ Hand R│
    └───────┘ └──┬───┘   └───┬──┘  └───────┘
             ankle_l      ankle_r
             (2 DOF)      (2 DOF)
             ┌──┴───┐    ┌───┴──┐
             │Foot L │    │Foot R│
             └───────┘    └──────┘
    
    Total: ~30 Degrees of Freedom
```

## Visualizing URDF

```bash
# View URDF in RViz2
ros2 launch urdf_tutorial display.launch.py model:=humanoid.urdf

# Check URDF for errors
check_urdf humanoid.urdf

# Generate visual graph
urdf_to_graphviz humanoid.urdf
```

## Using Xacro for Cleaner URDF

**Xacro** (XML Macros) makes URDF files more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Properties (variables) -->
  <xacro:property name="upper_arm_length" value="0.28"/>
  <xacro:property name="forearm_length" value="0.25"/>
  
  <!-- Macro for an arm segment -->
  <xacro:macro name="arm_segment" params="name length radius parent">
    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" 
                 iyy="0.01" iyz="0" izz="0.005"/>
      </inertial>
    </link>
    
    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <axis xyz="1 0 0"/>
      <limit lower="-2.0" upper="2.0" effort="50" velocity="2.0"/>
    </joint>
  </xacro:macro>
  
  <!-- Use the macro -->
  <xacro:arm_segment name="left_upper_arm" 
                     length="${upper_arm_length}" 
                     radius="0.04"
                     parent="torso_link"/>
  
  <xacro:arm_segment name="left_forearm" 
                     length="${forearm_length}" 
                     radius="0.035"
                     parent="left_upper_arm_link"/>
</robot>
```

## Loading URDF in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_state')
        
        # Load URDF from parameter
        self.declare_parameter('robot_description', '')
        
        # Publish joint states
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        
        self.timer = self.create_timer(0.02, self.publish_state)  # 50 Hz
    
    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'neck_pan', 'neck_tilt',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_roll', 'left_wrist_pitch',
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            # ... mirror for right side
        ]
        msg.position = [0.0] * len(msg.name)  # All joints at zero
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_pub.publish(msg)
```

## Summary

URDF is the foundational representation of robot structure in ROS 2. Understanding URDF is essential for simulation, visualization, control, and motion planning.

### Key Takeaways

- URDF defines the physical structure of a robot (links + joints)
- Links have visual, collision, and inertial properties
- Joint types match real mechanical connections
- Xacro provides variables, macros, and includes for cleaner files
- A humanoid typically has ~30 degrees of freedom

---

**Next:** [Module 2: Gazebo Simulation →](/docs/module-2/gazebo-simulation)
