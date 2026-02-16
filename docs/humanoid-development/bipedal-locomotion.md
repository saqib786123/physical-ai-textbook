---
sidebar_position: 1
title: "Bipedal Locomotion"
description: "Understanding and implementing bipedal walking for humanoid robots."
---

# Bipedal Locomotion

> *"Walking on two legs is so natural for humans that we forget it's one of the hardest problems in robotics."*

## The Challenge of Bipedal Walking

Bipedal locomotion requires:
- **Balance** — Maintaining center of mass over a small support polygon
- **Coordination** — Synchronizing 12+ leg joints in real-time
- **Adaptation** — Adjusting gait for uneven terrain
- **Energy efficiency** — Humans walk efficiently; robots often don't

## The Zero Moment Point (ZMP)

The ZMP is the fundamental concept for bipedal balance:

```python
import numpy as np

class ZMPController:
    """Zero Moment Point balance controller."""
    
    def compute_zmp(self, com_position, com_acceleration, foot_contacts):
        """
        Calculate where the ZMP is — if it's inside the support
        polygon (foot contact area), the robot won't fall.
        
        ZMP_x = COM_x - (COM_z * COM_accel_x) / (COM_accel_z + g)
        ZMP_y = COM_y - (COM_z * COM_accel_y) / (COM_accel_z + g)
        """
        g = 9.81
        
        zmp_x = com_position[0] - (
            com_position[2] * com_acceleration[0]
        ) / (com_acceleration[2] + g)
        
        zmp_y = com_position[1] - (
            com_position[2] * com_acceleration[1]
        ) / (com_acceleration[2] + g)
        
        return np.array([zmp_x, zmp_y])
    
    def is_balanced(self, zmp, support_polygon):
        """Check if ZMP is within the support polygon."""
        from shapely.geometry import Point, Polygon
        return Polygon(support_polygon).contains(Point(zmp))
```

## Walking Gait Phases

```
  Right Stance         Double Support        Left Stance
     Phase                Phase                Phase
       
    ┌─┐  ┌─┐           ┌─┐  ┌─┐            ┌─┐  ┌─┐
    │ │  │ │           │ │  │ │            │ │  │ │
    │ │  │ │           │ │  │ │            │ │  │ │
    │ │  │ │           │ │  │ │            │ │  │ │
    │ │  │ │           │ │  │ │            │ │  │ │
    └─┘  └ ┘           └─┘  └─┘            └ ┘  └─┘
    ████                ████  ████               ████
  (Right foot          (Both feet             (Left foot
   on ground,           on ground)             on ground,
   left swinging)                              right swinging)
```

### Gait Parameters

| Parameter | Typical Value | Description |
|-----------|--------------|-------------|
| Step length | 0.3-0.6 m | Distance between feet |
| Step height | 0.05-0.15 m | How high the foot lifts |
| Stride time | 0.8-1.2 s | Time for full stride cycle |
| Double support | 20-30% | Both feet on ground |

## Reinforcement Learning for Walking

```python
class BipedalWalkingReward:
    """Reward function for RL-based walking."""
    
    def compute(self, robot_state):
        reward = 0.0
        
        # Forward velocity reward
        reward += 2.0 * robot_state.base_linear_velocity[0]
        
        # Upright bonus
        upright = robot_state.base_orientation_euler[1]  # pitch
        reward += 1.0 * (1.0 - abs(upright) / 0.5)
        
        # Smooth motion penalty (discourage jerky movements)
        joint_accel = np.diff(robot_state.joint_velocities)
        reward -= 0.01 * np.sum(np.square(joint_accel))
        
        # Energy efficiency
        power = np.sum(np.abs(
            robot_state.joint_torques * robot_state.joint_velocities
        ))
        reward -= 0.001 * power
        
        # Foot contact pattern (encourage alternating steps)
        left_contact = robot_state.left_foot_contact
        right_contact = robot_state.right_foot_contact
        if left_contact != right_contact:
            reward += 0.5  # Reward single-foot stance (walking)
        
        # Termination (falling)
        if robot_state.base_height < 0.3:
            reward -= 10.0  # Heavy penalty for falling
        
        return reward
```

## Summary

Bipedal locomotion is one of the grand challenges of robotics. RL-based approaches trained in simulation are achieving increasingly natural and robust walking behaviors.

---

**Next:** [Dexterous Manipulation →](/docs/humanoid-development/dexterous-manipulation)
