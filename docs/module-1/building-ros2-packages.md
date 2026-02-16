---
sidebar_position: 4
title: "Building ROS 2 Packages"
description: "Learn to create, build, and manage ROS 2 Python packages with colcon."
keywords: [ROS 2 packages, colcon, ament, Python ROS 2, package management]
---

# Building ROS 2 Packages

> *"A ROS 2 package is the fundamental unit of organization — every piece of robot software lives in a package."*

## What is a ROS 2 Package?

A **package** is a directory containing related code, data, and configuration. Every ROS 2 node, library, or tool lives inside a package.

## Creating a Python Package

```bash
# Navigate to your workspace's src directory
cd ~/ros2_ws/src

# Create a new Python package
ros2 pkg create --build-type ament_python \
  --node-name physical_ai_node \
  physical_ai_perception \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

### Package Structure

```
physical_ai_perception/
├── package.xml                 # Package metadata & dependencies
├── setup.py                    # Python package configuration
├── setup.cfg                   # Entry point configuration
├── resource/
│   └── physical_ai_perception  # Marker file
├── physical_ai_perception/     # Python module
│   ├── __init__.py
│   └── physical_ai_node.py     # Your node
└── test/                       # Unit tests
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>physical_ai_perception</name>
  <version>0.1.0</version>
  <description>AI perception pipeline for Physical AI humanoid robots</description>
  <maintainer email="student@panaversity.org">Student Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py

```python
from setuptools import find_packages, setup

package_name = 'physical_ai_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch',
            ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@panaversity.org',
    description='Physical AI perception pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = physical_ai_perception.camera_node:main',
            'detector_node = physical_ai_perception.detector_node:main',
            'planner_node = physical_ai_perception.planner_node:main',
        ],
    },
)
```

## Building with Colcon

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build all packages
colcon build

# Build a specific package
colcon build --packages-select physical_ai_perception

# Build with symlink install (for rapid development)
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run your node
ros2 run physical_ai_perception camera_node
```

### Build Troubleshooting

```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Build with verbose output
colcon build --event-handlers console_direct+

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Workspace Organization

For this course, organize your workspace like this:

```
~/physical_ai_ws/
├── src/
│   ├── physical_ai_perception/    # Camera, LiDAR processing
│   ├── physical_ai_navigation/    # Path planning, SLAM
│   ├── physical_ai_control/       # Motor control, balance
│   ├── physical_ai_planning/      # Task planning with LLMs
│   ├── physical_ai_voice/         # Whisper integration
│   ├── humanoid_description/      # URDF, meshes
│   └── humanoid_bringup/          # Launch files, configs
└── install/
└── build/
└── log/
```

## Summary

- Use `ros2 pkg create` to scaffold packages
- `package.xml` defines dependencies
- `setup.py` configures entry points
- `colcon build` compiles everything
- Use `--symlink-install` during development for fast iteration

---

**Next:** [Launch Files and Parameters →](/docs/module-1/launch-files-parameters)
