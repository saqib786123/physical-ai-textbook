---
sidebar_position: 5
title: "Launch Files and Parameters"
description: "Using ROS 2 launch files to orchestrate multi-node systems and manage parameters."
keywords: [ROS 2 launch files, parameters, configuration, launch system]
---

# Launch Files and Parameters

> *"Launch files are the conductor of your robot orchestra — coordinating every node to play in harmony."*

## Why Launch Files?

A real robot system has **dozens of nodes**. Starting each one manually is impractical. Launch files let you:

- Start multiple nodes with one command
- Pass parameters and configurations
- Set up remappings (renaming topics)
- Include other launch files
- Conditionally launch nodes

## Python Launch Files

```python
# launch/full_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use simulation instead of real hardware'
    )
    
    return LaunchDescription([
        use_sim,
        
        # Camera Node
        Node(
            package='physical_ai_perception',
            executable='camera_node',
            name='camera',
            parameters=[{
                'camera_index': 0,
                'fps': 30,
                'resolution_width': 640,
                'resolution_height': 480,
            }],
            remappings=[
                ('/camera/image_raw', '/robot/camera/rgb'),
            ],
            output='screen',
        ),
        
        # AI Detection Node
        Node(
            package='physical_ai_perception',
            executable='detector_node',
            name='ai_detector',
            parameters=[{
                'model': 'yolov8n',
                'confidence_threshold': 0.5,
                'device': 'cuda',
            }],
            output='screen',
        ),
        
        # Navigation Node
        Node(
            package='physical_ai_navigation',
            executable='planner_node',
            name='path_planner',
            parameters=[
                'config/nav_params.yaml',
            ],
            output='screen',
        ),
    ])
```

### Running Launch Files

```bash
# Run a launch file
ros2 launch physical_ai_perception full_robot_launch.py

# With arguments
ros2 launch physical_ai_perception full_robot_launch.py use_sim:=false

# With verbose output
ros2 launch physical_ai_perception full_robot_launch.py --show-all-subprocesses
```

## Parameters

Parameters configure node behavior without code changes:

### YAML Configuration

```yaml
# config/robot_params.yaml
camera_node:
  ros__parameters:
    camera_index: 0
    fps: 30
    resolution:
      width: 640
      height: 480
    
detector_node:
  ros__parameters:
    model: "yolov8n"
    confidence_threshold: 0.5
    classes:
      - "person"
      - "cup"
      - "bottle"
      - "chair"
    
navigation_node:
  ros__parameters:
    max_velocity: 0.5          # m/s
    max_angular_velocity: 1.0  # rad/s
    safety_distance: 0.3       # meters
    planner: "NavfnPlanner"
```

### Using Parameters in Code

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare parameters with defaults
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_mode', True)
        self.declare_parameter('robot_name', 'humanoid_01')
        
        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_mode = self.get_parameter('safety_mode').value
        self.robot_name = self.get_parameter('robot_name').value
        
        self.get_logger().info(
            f'Robot "{self.robot_name}" initialized '
            f'(max_speed={self.max_speed}, safety={self.safety_mode})'
        )
        
        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.param_callback)
    
    def param_callback(self, params):
        """Called when parameters are changed at runtime."""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
            if param.name == 'max_speed':
                self.max_speed = param.value
        
        return SetParametersResult(successful=True)
```

### Runtime Parameter Changes

```bash
# List parameters
ros2 param list /configurable_node

# Get a parameter value
ros2 param get /configurable_node max_speed

# Set a parameter at runtime
ros2 param set /configurable_node max_speed 0.5

# Dump all parameters to YAML
ros2 param dump /configurable_node
```

## Summary

Launch files and parameters are essential for managing complex multi-node robot systems. They provide the organizational structure that makes large robotics projects maintainable.

---

**Next:** [Python Bridge with rclpy →](/docs/module-1/rclpy-python-bridge)
