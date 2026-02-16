---
sidebar_position: 7
title: "Reinforcement Learning for Robotics"
description: "Using RL to train robot locomotion and manipulation policies."
---

# Reinforcement Learning for Robotics

> *"RL lets robots learn by doing — discovering solutions humans never imagined."*

## Why RL for Robotics?

Some robot behaviors are too complex to program manually. Bipedal walking, for example, involves coordinating 12+ joints in real-time while maintaining balance. RL discovers these policies through trial and error.

## The RL Framework

```python
class RobotRLEnvironment:
    """Standard RL environment for robot training."""
    
    def __init__(self):
        self.observation_space = {
            'joint_positions': 30,    # 30 DOF humanoid
            'joint_velocities': 30,
            'imu_orientation': 4,     # quaternion
            'imu_angular_vel': 3,
            'foot_contacts': 2,       # left, right
        }
        self.action_space = 30  # Torque for each joint
    
    def step(self, action):
        """Apply action, advance physics, return results."""
        # Apply joint torques
        self.robot.apply_torques(action)
        
        # Step physics simulation
        self.sim.step(dt=0.002)
        
        # Get new observation
        obs = self.get_observation()
        
        # Calculate reward
        reward = self.compute_reward()
        
        # Check if episode is done
        done = self.is_fallen() or self.time > self.max_time
        
        return obs, reward, done, {}
    
    def compute_reward(self):
        """Reward function for humanoid walking."""
        # Forward velocity reward
        forward_reward = self.robot.base_velocity[0] * 2.0
        
        # Upright reward (penalize tilting)
        upright_reward = self.robot.base_orientation[3]  # w of quaternion
        
        # Energy penalty (prefer efficient movements)
        energy_penalty = -0.001 * sum(abs(self.robot.joint_torques))
        
        # Alive bonus
        alive_bonus = 1.0
        
        return forward_reward + upright_reward + energy_penalty + alive_bonus
```

## Training with Isaac Lab

```python
from omni.isaac.lab.envs import ManagerBasedRLEnv
import torch

# Configure training environment
env_cfg = HumanoidWalkingEnvCfg()
env_cfg.scene.num_envs = 4096  # 4096 parallel robots!

# Create environment
env = ManagerBasedRLEnv(cfg=env_cfg)

# PPO training loop (simplified)
policy = PolicyNetwork(obs_dim=69, act_dim=30)
optimizer = torch.optim.Adam(policy.parameters(), lr=3e-4)

for iteration in range(10000):
    # Collect rollouts from ALL environments
    observations = env.reset()
    
    for step in range(256):
        actions = policy(observations)
        observations, rewards, dones, infos = env.step(actions)
    
    # Update policy
    loss = compute_ppo_loss(rollout_data)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    
    print(f"Iteration {iteration}: Avg Reward = {rewards.mean():.2f}")
```

## Common RL Algorithms for Robotics

| Algorithm | Type | Best For |
|-----------|------|----------|
| **PPO** | On-policy | Locomotion, general control |
| **SAC** | Off-policy | Manipulation, dexterous tasks |
| **TD3** | Off-policy | Continuous control |
| **DDPG** | Off-policy | Simple motor control |
| **RSL-RL** | On-policy | Bipedal locomotion |

## Summary

RL enables robots to learn complex behaviors that are impossible to manually program. With GPU-parallel simulation in Isaac Lab, training that would take years in the real world takes hours.

---

**Next:** [Sim-to-Real Transfer →](/docs/module-3/sim-to-real-transfer)
