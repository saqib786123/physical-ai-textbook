---
sidebar_position: 1
title: "Capstone Project Overview"
description: "Build a complete Physical AI humanoid robot system from perception to action."
---

# Capstone Project: Autonomous Humanoid Assistant

> *"Put everything together — build a humanoid robot that can see, think, and act in a real-world scenario."*

## Project Goal

Build an **autonomous humanoid assistant** that can:

1. **Navigate** to a designated location in a room
2. **Identify** a target object using computer vision
3. **Plan** a grasp strategy using AI
4. **Manipulate** the object (pick up and place)
5. **Communicate** with a human user via natural language
6. **Maintain balance** throughout the task

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  Capstone System Architecture                  │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐ │
│  │  Perception   │    │  AI Planning   │    │   Execution     │ │
│  │               │    │                │    │                 │ │
│  │ • Camera      │───▶│ • LLM Planner  │───▶│ • Nav2         │ │
│  │ • LiDAR       │    │ • VLA Model    │    │ • MoveIt2      │ │
│  │ • IMU         │    │ • Task Graph   │    │ • Balance Ctrl │ │
│  │ • F/T Sensor  │    │                │    │ • Gripper Ctrl │ │
│  └─────────────┘    └──────────────┘    └─────────────────┘ │
│         │                   │                     │          │
│         └───────────────────┼─────────────────────┘          │
│                             │                                 │
│                    ┌────────▼────────┐                        │
│                    │   ROS 2 + DDS    │                        │
│                    │  Communication   │                        │
│                    └─────────────────┘                        │
└──────────────────────────────────────────────────────────────┘
```

## Phase 1: Simulation Setup (Week 1-2)

```python
# Set up the complete simulation environment
# 1. Create a realistic room in Gazebo/Isaac Sim
# 2. Spawn the humanoid robot
# 3. Add manipulable objects (cups, books, tools)
# 4. Configure all sensors (camera, LiDAR, IMU)
# 5. Test basic navigation and manipulation
```

### Deliverables
- [ ] Gazebo world file with furnished room
- [ ] URDF of selected humanoid robot
- [ ] All sensor topics publishing correctly
- [ ] Basic teleoperation working

## Phase 2: Perception Stack (Week 3-4)

- Integrate Isaac ROS for GPU-accelerated perception
- Set up SLAM for environment mapping
- Configure object detection pipeline
- Implement grasp pose estimation

### Deliverables
- [ ] Real-time object detection (>15 FPS)
- [ ] Complete room map via SLAM
- [ ] Grasp pose estimation for target objects
- [ ] Visualization in RViz2

## Phase 3: AI Brain (Week 5-6)

- Deploy VLA model for manipulation
- Integrate LLM for task planning
- Implement voice command interface
- Add error recovery and replanning

### Deliverables
- [ ] VLA model controlling manipulation
- [ ] LLM-based task decomposition working
- [ ] Voice commands functional
- [ ] Error recovery demonstrated

## Phase 4: Integration & Demo (Week 7-8)

- Full system integration
- End-to-end testing
- Performance optimization
- Documentation and presentation

### Deliverables
- [ ] Complete end-to-end demo
- [ ] Performance metrics report
- [ ] Code documentation
- [ ] Video demonstration

## Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| **Functionality** | 30% | Does the system work end-to-end? |
| **AI Integration** | 25% | Quality of AI/ML components |
| **Robustness** | 20% | Error handling, recovery |
| **Code Quality** | 15% | Clean, documented, modular |
| **Presentation** | 10% | Demo, documentation, video |

## Getting Started

```bash
# Clone the capstone starter repository
git clone https://github.com/panaversity/physical-ai-capstone.git
cd physical-ai-capstone

# Set up the workspace
mkdir -p ~/capstone_ws/src
cp -r packages/* ~/capstone_ws/src/

# Build
cd ~/capstone_ws
colcon build --symlink-install
source install/setup.bash

# Launch the simulation
ros2 launch capstone_bringup full_system.launch.py
```

## Summary

The capstone project integrates everything from the course into a single, working system. It demonstrates the full Physical AI pipeline: perception → reasoning → action.

Good luck, and welcome to the future of robotics! 🤖

---

**Back to:** [📘 Introduction](/docs/intro)
