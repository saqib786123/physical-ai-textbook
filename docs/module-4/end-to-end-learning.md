---
sidebar_position: 4
title: "End-to-End Learning"
description: "Training robot policies that go directly from sensor input to motor output."
---

# End-to-End Learning

> *"Skip the pipeline — learn the entire sensor-to-motor mapping in one neural network."*

## Traditional vs End-to-End

```
Traditional Pipeline:
Camera → [Perception] → [Planning] → [Control] → Motors
         Separate       Separate      Separate
         module         module        module

End-to-End:
Camera → [One Neural Network] → Motors
         Learns everything
         together
```

## Imitation Learning (Learning from Demonstration)

The most common approach for end-to-end robot learning:

```python
import torch
import torch.nn as nn
from torchvision import models

class ImitationPolicy(nn.Module):
    """Learn to imitate human demonstrations."""
    
    def __init__(self, action_dim=7):
        super().__init__()
        
        # Vision encoder (pre-trained ResNet)
        resnet = models.resnet18(pretrained=True)
        self.vision = nn.Sequential(*list(resnet.children())[:-1])
        
        # Action predictor
        self.action_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),  # Actions in [-1, 1]
        )
    
    def forward(self, image):
        features = self.vision(image).flatten(1)
        action = self.action_head(features)
        return action

# Training
policy = ImitationPolicy(action_dim=7)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)

for epoch in range(100):
    for images, expert_actions in dataloader:
        predicted_actions = policy(images)
        loss = F.mse_loss(predicted_actions, expert_actions)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

## Diffusion Policy

**Diffusion Policy** generates smooth, multi-modal action sequences:

```python
class DiffusionPolicy:
    """
    Generate robot actions using diffusion process.
    Can produce diverse, smooth trajectories.
    """
    
    def __init__(self, obs_dim, action_dim, horizon=16):
        self.horizon = horizon  # Predict 16 future actions
        self.noise_scheduler = DDPMScheduler(num_train_timesteps=100)
        self.model = ConditionalUNet1D(
            input_dim=action_dim,
            global_cond_dim=obs_dim,
            diffusion_step_embed_dim=128,
        )
    
    def predict(self, observation):
        """Generate action trajectory from observation."""
        # Start from noise
        noisy_action = torch.randn(1, self.horizon, self.action_dim)
        
        # Iteratively denoise
        for t in self.noise_scheduler.timesteps:
            noise_pred = self.model(
                noisy_action, t, 
                global_cond=observation
            )
            noisy_action = self.noise_scheduler.step(
                noise_pred, t, noisy_action
            ).prev_sample
        
        return noisy_action  # Clean action trajectory
```

## ACT (Action Chunking with Transformers)

```python
class ACTPolicy(nn.Module):
    """
    Action Chunking with Transformers.
    Predicts a chunk of future actions at once.
    """
    
    def __init__(self, chunk_size=100):
        super().__init__()
        self.chunk_size = chunk_size
        
        # Vision encoder
        self.backbone = models.resnet18(pretrained=True)
        
        # Transformer for action prediction
        self.transformer = nn.Transformer(
            d_model=512,
            nhead=8,
            num_encoder_layers=4,
            num_decoder_layers=4,
        )
        
        # Action queries (learnable)
        self.action_queries = nn.Parameter(
            torch.randn(chunk_size, 512)
        )
    
    def forward(self, images, joint_positions):
        # Encode visual observation
        vis_features = self.backbone(images)
        
        # Concatenate with proprioception
        obs = torch.cat([vis_features, joint_positions], dim=-1)
        
        # Decode action chunk
        actions = self.transformer(
            src=obs.unsqueeze(0),
            tgt=self.action_queries.unsqueeze(1),
        )
        
        return actions  # [chunk_size, action_dim]
```

## Summary

End-to-end learning eliminates the need for hand-designed perception, planning, and control pipelines. Methods like imitation learning, diffusion policies, and ACT provide powerful approaches for training robot behaviors directly from demonstrations.

---

**Next:** [Humanoid Locomotion →](/docs/humanoid-development/bipedal-locomotion)
