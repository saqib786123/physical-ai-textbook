---
sidebar_position: 2
title: "Dexterous Manipulation"
description: "Robot hand control and object manipulation strategies for humanoid robots."
---

# Dexterous Manipulation

> *"The human hand has 27 degrees of freedom and can manipulate virtually any object. Matching this in a robot is the ultimate challenge."*

## Types of Grasps

| Grasp Type | Fingers Used | Example Objects | Stability |
|------------|-------------|-----------------|-----------|
| **Power Grasp** | All 5 | Hammer, bottle | Very stable |
| **Precision Grasp** | 2-3 (fingertips) | Needle, coin | Less stable |
| **Pinch Grasp** | 2 (thumb + index) | Card, key | Moderate |
| **Cylindrical Grasp** | All 5 (wrap) | Cup, can | Stable |
| **Hook Grasp** | 4 fingers (no thumb) | Bag handle | Moderate |

## Grasp Planning

```python
import numpy as np

class GraspPlanner:
    """Plan grasps for detected objects."""
    
    def plan_grasp(self, object_pointcloud, object_class):
        """
        Generate grasp candidates for an object.
        
        Returns list of (position, orientation, width, score) tuples.
        """
        grasp_candidates = []
        
        # Get object dimensions
        bbox = self.compute_bounding_box(object_pointcloud)
        width = bbox['width']
        height = bbox['height']
        
        # Generate candidates based on object type
        if object_class in ['cup', 'bottle', 'can']:
            # Cylindrical grasp from the side
            grasps = self.generate_side_grasps(bbox, num=10)
            grasp_candidates.extend(grasps)
        
        if width < 0.08:  # Small enough for pinch grasp
            grasps = self.generate_top_grasps(bbox, num=10)
            grasp_candidates.extend(grasps)
        
        # Score each candidate
        scored = []
        for grasp in grasp_candidates:
            score = self.evaluate_grasp(
                grasp, object_pointcloud, 
                collision_check=True
            )
            scored.append((grasp, score))
        
        # Return best grasp
        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[0]
    
    def evaluate_grasp(self, grasp, pointcloud, collision_check=True):
        """Score a grasp candidate (0-1)."""
        score = 0.0
        
        # Antipodal quality (opposing contact normals)
        score += 0.4 * self.antipodal_quality(grasp, pointcloud)
        
        # Force closure (can resist external forces)
        score += 0.3 * self.force_closure_quality(grasp)
        
        # Reachability (can the arm reach this pose?)
        score += 0.2 * self.reachability_score(grasp)
        
        # Collision free
        if collision_check:
            if self.check_collision(grasp):
                score = 0.0
            else:
                score += 0.1
        
        return score
```

## Tactile Sensing for Manipulation

```python
class TactileGraspController:
    """Use tactile feedback to control grasp force."""
    
    def __init__(self, target_force=5.0):  # Newtons
        self.target_force = target_force
        self.kp = 0.1  # Proportional gain
        self.ki = 0.01  # Integral gain
        self.force_integral = 0.0
    
    def adaptive_grasp(self, tactile_readings):
        """
        Adjust gripper based on tactile feedback.
        Too little force → object slips.
        Too much force → object breaks.
        """
        current_force = np.mean(tactile_readings)
        
        # PI controller
        error = self.target_force - current_force
        self.force_integral += error
        
        gripper_adjustment = (
            self.kp * error + 
            self.ki * self.force_integral
        )
        
        # Slip detection
        if self.detect_slip(tactile_readings):
            gripper_adjustment += 0.5  # Increase grip
        
        return gripper_adjustment
    
    def detect_slip(self, readings):
        """Detect if object is slipping based on tactile vibration."""
        high_freq = np.abs(np.fft.fft(readings))[5:]
        return np.max(high_freq) > self.slip_threshold
```

## AI-Powered Manipulation

```python
class VLAManipulator:
    """Use a VLA model for manipulation."""
    
    def pick_and_place(self, instruction):
        """
        Execute a pick-and-place task from natural language.
        Example: "Pick up the red cup and place it on the table"
        """
        image = self.camera.capture()
        
        # VLA predicts continuous action sequence
        while not self.task_complete(instruction):
            image = self.camera.capture()
            action = self.vla_model.predict(image, instruction)
            
            # Action: [dx, dy, dz, drx, dry, drz, gripper]
            self.robot.move_relative(
                position=action[:3],
                rotation=action[3:6],
            )
            
            if action[6] > 0.5:
                self.robot.open_gripper()
            else:
                self.robot.close_gripper()
```

## Summary

Dexterous manipulation combines grasp planning, force control, tactile sensing, and increasingly, AI-based approaches to enable robots to handle the vast variety of objects in human environments.

---

**Next:** [Whole-Body Control →](/docs/humanoid-development/whole-body-control)
