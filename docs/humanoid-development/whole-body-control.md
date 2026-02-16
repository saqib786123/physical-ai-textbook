---
sidebar_position: 3
title: "Whole-Body Control"
description: "Coordinating all joints of a humanoid robot for complex tasks."
---

# Whole-Body Control

> *"A humanoid robot must coordinate 30+ joints simultaneously — walking, reaching, balancing, all at once."*

## The Whole-Body Control Problem

Humanoid robots must:
- Walk while carrying objects
- Reach for items while maintaining balance
- React to pushes while performing tasks
- Coordinate arms, legs, torso, and head simultaneously

## Task Priority Framework

```python
class WholeBodyController:
    """
    Prioritized whole-body control using task hierarchy.
    
    Priority levels:
    1. Balance (highest - never compromise)
    2. Collision avoidance
    3. End-effector task (reaching, grasping)
    4. Posture (lowest - nice to have)
    """
    
    def __init__(self, robot):
        self.robot = robot
        self.tasks = []
    
    def add_task(self, task, priority):
        self.tasks.append((priority, task))
        self.tasks.sort(key=lambda x: x[0])
    
    def solve(self):
        """Solve for joint velocities respecting task priorities."""
        q_dot = np.zeros(self.robot.num_joints)
        null_space = np.eye(self.robot.num_joints)
        
        for priority, task in self.tasks:
            J = task.get_jacobian()
            x_dot_desired = task.get_desired_velocity()
            
            # Project into null space of higher-priority tasks
            J_proj = J @ null_space
            
            # Solve for this task's contribution
            J_pinv = np.linalg.pinv(J_proj)
            q_dot += J_pinv @ (x_dot_desired - J @ q_dot)
            
            # Update null space
            null_space = null_space @ (np.eye(len(q_dot)) - J_pinv @ J_proj)
        
        return q_dot
```

## Balance + Manipulation Example

```python
class BalancedReach:
    """Reach for an object while maintaining balance."""
    
    def reach_while_balanced(self, target_position):
        wbc = WholeBodyController(self.robot)
        
        # Priority 1: Keep COM over feet
        balance_task = COMBalanceTask(
            desired_com=self.compute_stable_com(),
            weight=10.0,
        )
        wbc.add_task(balance_task, priority=1)
        
        # Priority 2: Avoid self-collision
        collision_task = SelfCollisionAvoidance(
            min_distance=0.05,
        )
        wbc.add_task(collision_task, priority=2)
        
        # Priority 3: Reach target with right hand
        reach_task = EndEffectorTask(
            link='right_hand',
            target_position=target_position,
        )
        wbc.add_task(reach_task, priority=3)
        
        # Priority 4: Keep torso upright
        posture_task = PostureTask(
            desired_joint_angles=self.neutral_pose,
            weight=0.1,
        )
        wbc.add_task(posture_task, priority=4)
        
        # Solve and execute
        joint_velocities = wbc.solve()
        self.robot.set_joint_velocities(joint_velocities)
```

## MoveIt2 for Motion Planning

```python
from moveit2 import MoveIt2

class HumanoidMotionPlanner:
    def __init__(self):
        self.moveit = MoveIt2(
            node=self.node,
            joint_names=self.arm_joint_names,
            base_link_name="torso_link",
            end_effector_name="right_hand_link",
        )
    
    def plan_and_execute(self, target_pose):
        """Plan a collision-free path and execute it."""
        # Set target
        self.moveit.set_pose_goal(target_pose)
        
        # Plan (finds collision-free path)
        success = self.moveit.plan()
        
        if success:
            # Execute the plan
            self.moveit.execute()
        else:
            self.get_logger().warn("No valid path found!")
```

## Summary

Whole-body control is essential for humanoid robots that need to perform tasks while maintaining balance and avoiding collisions. The task priority framework ensures critical behaviors like balance are never compromised.

---

**Next:** [Capstone Project →](/docs/capstone/capstone-overview)
