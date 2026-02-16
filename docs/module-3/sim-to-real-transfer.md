---
sidebar_position: 8
title: "Sim-to-Real Transfer"
description: "Techniques for transferring robot behaviors trained in simulation to real hardware."
---

# Sim-to-Real Transfer

> *"The ultimate test: can your simulated robot's behavior survive contact with reality?"*

## The Sim-to-Real Gap

Models trained in simulation often fail in the real world due to differences between simulation and reality:

| Factor | Simulation | Real World |
|--------|-----------|------------|
| Physics | Approximate | Exact (by definition) |
| Sensors | Perfect + noise model | Imperfect + unknown noise |
| Actuators | Instant, precise | Delayed, noisy, wear |
| Lighting | Controlled | Variable |
| Materials | Simplified | Complex |

## Techniques for Bridging the Gap

### 1. Domain Randomization

Randomize simulation parameters so the model learns to be robust:

```python
class DomainRandomizer:
    def randomize(self, env):
        # Physics randomization
        env.gravity = 9.81 + np.random.uniform(-0.5, 0.5)
        env.friction = np.random.uniform(0.4, 1.0)
        env.joint_damping *= np.random.uniform(0.8, 1.2)
        
        # Visual randomization
        env.set_lighting(np.random.uniform(200, 2000))
        env.set_texture(random.choice(textures))
        env.set_camera_noise(np.random.uniform(0, 0.02))
        
        # Dynamics randomization
        env.mass *= np.random.uniform(0.9, 1.1)
        env.motor_strength *= np.random.uniform(0.85, 1.15)
        env.sensor_delay = np.random.uniform(0, 0.02)  # 0-20ms
```

### 2. System Identification

Measure real robot parameters and update simulation:

```python
def system_identification(real_robot, sim_robot):
    """Match simulation to reality."""
    
    # Measure real joint friction
    for joint in real_robot.joints:
        real_friction = measure_friction(joint)
        sim_robot.set_joint_friction(joint.name, real_friction)
    
    # Measure real motor response
    for motor in real_robot.motors:
        real_response = measure_step_response(motor)
        sim_robot.set_motor_model(motor.name, real_response)
    
    # Calibrate sensors
    for sensor in real_robot.sensors:
        real_noise = measure_sensor_noise(sensor)
        sim_robot.set_sensor_noise(sensor.name, real_noise)
```

### 3. Progressive Training Pipeline

```
Phase 1: Pure Simulation
├── Train on 4096 parallel environments
├── Domain randomization active
└── Achieve stable behavior

Phase 2: Fine-tune on Real Data
├── Collect small dataset from real robot
├── Fine-tune model on real data
└── 10-100x less data needed

Phase 3: Real-World Deployment
├── Deploy to Jetson/robot
├── Monitor performance
└── Collect failure cases → retrain
```

## Deployment to Jetson

```python
import tensorrt as trt
import numpy as np

class JetsonInference:
    """Run trained policy on NVIDIA Jetson."""
    
    def __init__(self, engine_path):
        # Load TensorRT engine
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        
        self.context = self.engine.create_execution_context()
    
    def predict(self, observation):
        """Run inference at >100 Hz on Jetson."""
        # Allocate buffers
        input_buf = np.array(observation, dtype=np.float32)
        output_buf = np.zeros(self.action_dim, dtype=np.float32)
        
        # Execute
        self.context.execute_v2([input_buf.ctypes.data, output_buf.ctypes.data])
        
        return output_buf  # Joint torques/positions
```

## Summary

Sim-to-real transfer is the bridge between virtual training and physical deployment. Domain randomization, system identification, and progressive training make this bridge robust.

---

**Next:** [Module 4: Vision-Language-Action →](/docs/module-4/vision-language-action)
