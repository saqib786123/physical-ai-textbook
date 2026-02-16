---
sidebar_position: 6
title: "Human-Robot Interaction Simulation"
description: "Simulating natural human-robot interaction scenarios for safe testing and development."
keywords: [human-robot interaction, HRI, social robotics, safety, interaction design]
---

# Human-Robot Interaction Simulation

> *"Before a robot can safely interact with humans, every interaction must be tested thousands of times in simulation."*

## Why Simulate HRI?

Human-Robot Interaction (HRI) involves safety-critical scenarios. Testing these in the real world is dangerous and impractical during development.

## Key HRI Scenarios to Simulate

### 1. Personal Space Management

```python
class PersonalSpaceController:
    """Maintain appropriate distance from humans."""
    
    INTIMATE = 0.45    # meters (too close!)
    PERSONAL = 1.2     # meters (conversation distance)
    SOCIAL = 3.6       # meters (group interaction)
    PUBLIC = 7.6       # meters (presentation)
    
    def get_appropriate_distance(self, context):
        if context == "handover":
            return self.PERSONAL * 0.8  # Need to be close for object handover
        elif context == "conversation":
            return self.PERSONAL
        elif context == "navigation":
            return self.SOCIAL
        else:
            return self.PUBLIC
    
    def adjust_position(self, robot_pose, human_pose, context):
        desired_distance = self.get_appropriate_distance(context)
        current_distance = self.compute_distance(robot_pose, human_pose)
        
        if current_distance < self.INTIMATE:
            return "RETREAT"  # Too close — back away immediately
        elif current_distance < desired_distance * 0.8:
            return "BACK_UP"  # Slightly too close
        elif current_distance > desired_distance * 1.5:
            return "APPROACH"  # Can move closer
        else:
            return "HOLD"  # Good distance
```

### 2. Safe Object Handover

```python
class HandoverController:
    """Control safe object handover between robot and human."""
    
    def execute_handover(self, object_in_gripper):
        """
        Steps for safe handover:
        1. Approach human at appropriate speed
        2. Extend arm with object
        3. Wait for human to grasp
        4. Detect human grip (force sensors)
        5. Release object
        6. Retract arm
        """
        # Approach slowly
        self.move_to_handover_position(speed=0.2)  # m/s (slow!)
        
        # Extend arm
        self.extend_arm_for_handover()
        
        # Wait for human to grasp
        while not self.detect_human_grip():
            if self.human_moved_away():
                self.retract_arm()
                return "CANCELLED"
        
        # Gradually release
        self.open_gripper(speed=0.5)  # Open slowly
        
        # Verify handover
        if self.object_still_held():
            return "SUCCESS"
        else:
            return "DROPPED"  # Object fell!
```

### 3. Collision Avoidance with Humans

```python
class HumanSafetySystem:
    """Ensure robot never collides with humans."""
    
    SAFETY_ZONES = {
        'danger': 0.3,     # meters — STOP immediately
        'warning': 1.0,    # meters — Slow down significantly
        'caution': 2.0,    # meters — Reduce speed
        'normal': float('inf'),  # Full speed
    }
    
    def get_speed_limit(self, min_human_distance):
        if min_human_distance < self.SAFETY_ZONES['danger']:
            return 0.0  # FULL STOP
        elif min_human_distance < self.SAFETY_ZONES['warning']:
            return 0.1  # Very slow
        elif min_human_distance < self.SAFETY_ZONES['caution']:
            return 0.3  # Cautious
        else:
            return 1.0  # Normal speed
```

## Simulating Humans in Gazebo

```xml
<!-- Add an animated human actor in Gazebo -->
<actor name="walking_human">
  <skin>
    <filename>walk.dae</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>true</loop>
    <auto_start>true</auto_start>
    <trajectory id="0" type="walking">
      <waypoint>
        <time>0</time>
        <pose>0 2 1.0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>5</time>
        <pose>5 2 1.0 0 0 0</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

## Summary

HRI simulation is critical for developing robots that can safely and naturally interact with humans. Simulation allows testing thousands of interaction scenarios before deployment.

### Key Takeaways

- Test all human-facing behaviors in simulation first
- Personal space management is a fundamental HRI requirement
- Object handover requires careful force control
- Safety zones with speed limiting prevent collisions
- Animated human actors in Gazebo enable realistic testing

---

**Next:** [Module 3: NVIDIA Isaac Platform →](/docs/module-3/nvidia-isaac-platform)
