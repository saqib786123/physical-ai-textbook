---
sidebar_position: 1
title: "Vision-Language-Action Models"
description: "Understanding VLA models that combine vision, language understanding, and robotic action."
---

# Vision-Language-Action (VLA) Models

> *"VLA models give robots the ability to see the world, understand language commands, and act accordingly — all in one unified model."*

## The VLA Revolution

VLA models represent the convergence of three AI capabilities that were previously separate:

```
┌─────────────────────────────────────────────────────────┐
│              Vision-Language-Action (VLA)                 │
│                                                          │
│  ┌──────────┐   ┌───────────┐   ┌─────────────┐        │
│  │  Vision   │ + │ Language   │ + │   Action     │       │
│  │           │   │            │   │              │       │
│  │ See the   │   │ Understand │   │ Move the     │       │
│  │ world     │   │ commands   │   │ robot        │       │
│  │           │   │            │   │              │       │
│  │ Camera    │   │ "Pick up   │   │ Joint angles │       │
│  │ images    │   │  the red   │   │ & torques    │       │
│  │           │   │  cup"      │   │              │       │
│  └──────────┘   └───────────┘   └─────────────┘        │
│                                                          │
│  Input: Image + Text ──────────▶ Output: Robot Actions  │
└─────────────────────────────────────────────────────────┘
```

## Key VLA Models

| Model | Developer | Released | Key Innovation |
|-------|-----------|----------|----------------|
| **RT-2** | Google DeepMind | 2023 | First large VLA (55B params) |
| **Octo** | UC Berkeley | 2024 | Open-source, generalizable |
| **OpenVLA** | Stanford | 2024 | Open-source 7B VLA |
| **π₀** | Physical Intelligence | 2024 | Flow matching, dexterous |
| **GR00T** | NVIDIA | 2024 | Humanoid-specific |
| **RT-X** | Open X-Embodiment | 2024 | Cross-robot training data |

## How VLA Models Work

### Architecture

```
Camera Image ──┐
               ├──▶ Vision Encoder ──┐
Depth Image ───┘    (ViT/SigLIP)    │
                                     ├──▶ Language Model ──▶ Action Tokens
Text Command ──────▶ Tokenizer ─────┘    (LLaMA/Gemma)     │
                                                             │
                                           Action Decoder ◀──┘
                                           (De-tokenize)
                                                │
                                                ▼
                                        Joint Commands
                                     [θ₁, θ₂, ... θₙ, gripper]
```

### Pipeline in Code

```python
import torch
from transformers import AutoModelForVision2Seq, AutoProcessor

class VLAController:
    """Control a robot using a Vision-Language-Action model."""
    
    def __init__(self, model_name="openvla/openvla-7b"):
        self.processor = AutoProcessor.from_pretrained(model_name)
        self.model = AutoModelForVision2Seq.from_pretrained(
            model_name,
            torch_dtype=torch.bfloat16,
            device_map="auto",
        )
    
    def predict_action(self, image, instruction):
        """
        Given a camera image and language instruction,
        predict the next robot action.
        
        Args:
            image: PIL Image from robot camera
            instruction: str, e.g. "Pick up the red cup"
        
        Returns:
            action: [x, y, z, rx, ry, rz, gripper] (7-DOF)
        """
        # Prepare inputs
        prompt = f"In: What action should the robot take to {instruction}?\nOut:"
        inputs = self.processor(prompt, image).to("cuda")
        
        # Generate action tokens
        action_tokens = self.model.generate(
            **inputs,
            do_sample=False,
            max_new_tokens=256,
        )
        
        # Decode to robot actions
        action = self.processor.decode(
            action_tokens[0], skip_special_tokens=True
        )
        
        return self.parse_action(action)
    
    def parse_action(self, action_str):
        """Convert model output to robot commands."""
        # VLA models output actions as discretized tokens
        # Each dimension is binned into 256 values
        values = [float(x) for x in action_str.split(',')]
        
        # Normalize to robot action space
        return {
            'position': values[:3],      # [x, y, z] in meters
            'rotation': values[3:6],      # [rx, ry, rz] in radians
            'gripper': values[6],         # 0.0 (closed) to 1.0 (open)
        }
```

## Training VLA Models

### Data Collection

```python
class RobotDataCollector:
    """Collect demonstration data for VLA training."""
    
    def __init__(self):
        self.episodes = []
    
    def record_episode(self, task_description):
        """Record a human demonstration."""
        episode = {
            'task': task_description,
            'frames': [],
        }
        
        while not done:
            frame = {
                'image': self.camera.capture(),
                'instruction': task_description,
                'action': self.robot.get_current_action(),
                'joint_positions': self.robot.get_joint_positions(),
                'gripper_state': self.robot.get_gripper_state(),
                'timestamp': time.time(),
            }
            episode['frames'].append(frame)
        
        self.episodes.append(episode)
```

### Training Pipeline

```python
# Simplified VLA training
from torch.utils.data import DataLoader

# Load demonstration dataset (e.g., Open X-Embodiment)
dataset = RobotDemonstrationDataset(
    data_dir="robot_demonstrations/",
    image_size=(224, 224),
)

dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# Fine-tune the VLA model
optimizer = torch.optim.AdamW(model.parameters(), lr=2e-5)

for epoch in range(10):
    for batch in dataloader:
        images = batch['image'].to('cuda')
        instructions = batch['instruction']
        target_actions = batch['action'].to('cuda')
        
        # Forward pass
        predicted_actions = model(images, instructions)
        
        # MSE loss on action predictions
        loss = F.mse_loss(predicted_actions, target_actions)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    
    print(f"Epoch {epoch}: Loss = {loss.item():.4f}")
```

## Open X-Embodiment Dataset

The **Open X-Embodiment** project aggregates robot demonstration data from dozens of labs:

- **1M+ robot episodes** from 22 robot types
- Tasks: picking, placing, pushing, opening, pouring
- Standardized data format (RLDS/TFDS)

## Summary

VLA models represent the future of robot control — integrating perception, understanding, and action into a single neural network. They enable robots to follow natural language commands and generalize to new tasks.

### Key Takeaways

- VLA models combine vision, language, and action in one model
- They take camera images + text commands → robot actions
- Training requires large demonstration datasets
- Open-source models (OpenVLA, Octo) make this accessible
- VLA enables generalization to new objects and tasks

---

**Next:** [Multimodal Foundation Models →](/docs/module-4/multimodal-foundation-models)
