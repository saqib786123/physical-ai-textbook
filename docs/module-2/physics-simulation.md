---
sidebar_position: 3
title: "Physics Simulation"
description: "Understanding physics engines, rigid body dynamics, and collision handling in robot simulation."
keywords: [physics simulation, rigid body dynamics, collision detection, ODE, physics engine]
---

# Physics Simulation

> *"The physics engine is the reality engine — it ensures your simulated robot behaves like a real one."*

## Why Physics Matters

Physical AI robots must understand and obey the laws of physics. A simulation without accurate physics is useless for developing real-world robots.

### Core Physics Concepts

| Concept | Description | Relevance to Robotics |
|---------|-------------|----------------------|
| **Gravity** | 9.81 m/s² downward | Affects balance, falling objects |
| **Friction** | Resistance to sliding | Foot traction, grasping |
| **Inertia** | Resistance to acceleration | Motor sizing, response time |
| **Momentum** | Mass × velocity | Impact forces, collisions |
| **Torque** | Rotational force | Joint motors, manipulation |

## Rigid Body Dynamics

### Newton-Euler Equations

Every link in a robot follows these fundamental equations:

```python
import numpy as np

class RigidBody:
    """Simplified rigid body dynamics simulation."""
    
    def __init__(self, mass, inertia_tensor, position, orientation):
        self.mass = mass
        self.I = inertia_tensor       # 3x3 inertia matrix
        self.position = position       # 3D position [x, y, z]
        self.orientation = orientation # Quaternion [w, x, y, z]
        self.velocity = np.zeros(3)    # Linear velocity
        self.omega = np.zeros(3)       # Angular velocity
    
    def apply_force(self, force, point_of_application):
        """Apply a force at a specific point on the body."""
        # Linear acceleration: F = ma → a = F/m
        linear_accel = force / self.mass
        
        # Torque: τ = r × F
        r = point_of_application - self.position
        torque = np.cross(r, force)
        
        # Angular acceleration: τ = Iα → α = I⁻¹τ
        angular_accel = np.linalg.solve(self.I, torque)
        
        return linear_accel, angular_accel
    
    def step(self, dt, forces, gravity=np.array([0, 0, -9.81])):
        """Advance simulation by one timestep."""
        # Sum all forces (including gravity)
        total_force = sum(forces) + gravity * self.mass
        
        # Compute accelerations
        linear_accel = total_force / self.mass
        
        # Semi-implicit Euler integration
        self.velocity += linear_accel * dt
        self.position += self.velocity * dt
```

## Collision Detection

### Broad Phase vs Narrow Phase

```
Broad Phase: "Which objects MIGHT be colliding?"
├── AABB (Axis-Aligned Bounding Box) overlap test
├── Very fast, many false positives
└── Reduces N² checks to manageable number

Narrow Phase: "Are these objects ACTUALLY colliding?"
├── GJK algorithm (convex shapes)
├── Mesh-mesh intersection (complex shapes)
└── Returns contact points, normals, penetration depth
```

### Contact Handling

```python
class ContactSolver:
    """Handle contacts between objects."""
    
    def resolve_contact(self, body_a, body_b, contact):
        """
        Resolve a collision contact between two bodies.
        
        Args:
            contact: Contains normal, penetration_depth, friction
        """
        # Relative velocity at contact point
        rel_vel = body_a.velocity - body_b.velocity
        
        # Normal impulse (prevents penetration)
        vel_along_normal = np.dot(rel_vel, contact.normal)
        
        if vel_along_normal > 0:
            return  # Bodies are separating
        
        # Coefficient of restitution (bounciness)
        e = min(body_a.restitution, body_b.restitution)
        
        # Impulse magnitude
        j = -(1 + e) * vel_along_normal
        j /= (1/body_a.mass + 1/body_b.mass)
        
        # Apply impulse
        impulse = j * contact.normal
        body_a.velocity += impulse / body_a.mass
        body_b.velocity -= impulse / body_b.mass
```

## Friction Models

Friction is critical for humanoid walking and object manipulation:

### Coulomb Friction

```python
def coulomb_friction(normal_force, mu_static, mu_dynamic, tangent_velocity):
    """
    Coulomb friction model:
    - Static friction: prevents motion when force < μs × N
    - Dynamic friction: opposes motion with force = μd × N
    """
    max_static = mu_static * abs(normal_force)
    dynamic_force = mu_dynamic * abs(normal_force)
    
    speed = np.linalg.norm(tangent_velocity)
    
    if speed < 0.001:  # Nearly static
        return max_static  # Can resist up to this force
    else:
        # Dynamic friction opposes motion
        direction = -tangent_velocity / speed
        return dynamic_force * direction
```

### Friction Values for Robotics

| Surface Pair | μ (static) | μ (dynamic) |
|--------------|-----------|-------------|
| Rubber on concrete | 0.8 | 0.6 |
| Rubber on wood | 0.7 | 0.5 |
| Metal on metal | 0.5 | 0.3 |
| Rubber on ice | 0.15 | 0.08 |

## Simulating in Gazebo

### Setting Physics Parameters

```xml
<physics type="ode">
  <!-- Step size: smaller = more accurate but slower -->
  <max_step_size>0.001</max_step_size>
  
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>  <!-- More iterations = more accurate -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.8</erp>
    </constraints>
  </ode>
</physics>
```

## Summary

Physics simulation is the foundation of Physical AI development. Understanding rigid body dynamics, collision handling, and friction models is essential for creating realistic simulations that transfer to real hardware.

### Key Takeaways

- Physics engines solve Newton's equations at each timestep
- Collision detection has broad phase (fast) and narrow phase (accurate)
- Friction (especially foot-ground) is critical for humanoid balance
- Step size trades accuracy for speed
- Inertial properties must be realistic for meaningful simulation

---

**Next:** [Sensor Simulation →](/docs/module-2/sensor-simulation)
