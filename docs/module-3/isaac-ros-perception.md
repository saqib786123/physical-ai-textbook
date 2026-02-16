---
sidebar_position: 4
title: "Isaac ROS Perception"
description: "Hardware-accelerated perception with Isaac ROS packages."
---

# Isaac ROS Perception

> *"Isaac ROS makes your robot see 10x faster — GPU-accelerating every perception pipeline."*

## Overview

Isaac ROS provides GPU-accelerated ROS 2 packages that replace CPU-bound standard packages. This is critical for humanoid robots that need to perceive and react in real-time.

## Key Perception Packages

### Object Detection with Isaac ROS

```python
# Launch Isaac ROS object detection
# Uses TensorRT for GPU-accelerated inference

# launch/perception.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # DNN Image Encoder
                ComposableNode(
                    package='isaac_ros_dnn_image_encoder',
                    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                    parameters=[{
                        'input_image_width': 640,
                        'input_image_height': 480,
                        'network_image_width': 640,
                        'network_image_height': 640,
                    }],
                ),
                # TensorRT Inference
                ComposableNode(
                    package='isaac_ros_tensor_rt',
                    plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                    parameters=[{
                        'model_file_path': 'models/yolov8n.onnx',
                        'engine_file_path': 'models/yolov8n.engine',
                        'input_binding_names': ['images'],
                        'output_binding_names': ['output0'],
                    }],
                ),
            ],
        ),
    ])
```

## Performance Comparison

| Pipeline | CPU (fps) | GPU Isaac ROS (fps) | Speedup |
|----------|-----------|-------------------|---------|
| Object Detection | 5-10 | 60-100 | **10x** |
| Visual SLAM | 15-20 | 60-90 | **4x** |
| Depth Processing | 10-15 | 120+ | **10x** |
| AprilTag Detection | 20-30 | 200+ | **8x** |
| Stereo Matching | 5-8 | 60-80 | **10x** |

## Summary

Isaac ROS transforms perception from a bottleneck into a strength. GPU acceleration enables real-time perception at framerates impossible on CPU alone.

---

**Next:** [Visual SLAM →](/docs/module-3/visual-slam)
