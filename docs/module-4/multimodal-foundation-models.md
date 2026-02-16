---
sidebar_position: 2
title: "Multimodal Foundation Models"
description: "Foundation models that understand vision, language, and spatial reasoning for robotics."
---

# Multimodal Foundation Models

> *"Foundation models give robots common-sense understanding of the world — something traditional robotics has always lacked."*

## What Are Foundation Models?

**Foundation models** are large neural networks trained on broad data that can be adapted to many downstream tasks. In robotics, they provide:

- **Visual understanding** — Recognize objects, scenes, spatial relationships
- **Language comprehension** — Parse natural language instructions
- **Common sense** — Understand how the physical world works
- **Zero-shot generalization** — Handle objects/tasks never seen during training

## Key Foundation Models for Robotics

| Model | Modalities | Size | Use in Robotics |
|-------|-----------|------|-----------------|
| **GPT-4o** | Text + Vision | ~1.8T | Task planning, scene understanding |
| **Gemini 2.0** | Text + Vision + Audio | Large | Multimodal reasoning |
| **CLIP** | Vision + Text | 400M | Object recognition, scene matching |
| **SAM 2** | Vision | 600M | Object segmentation (any object) |
| **DINOv2** | Vision | 1.1B | Visual features for manipulation |
| **SigLIP** | Vision + Text | 400M | Image-text alignment (used in VLAs) |
| **Whisper** | Audio | 1.5B | Voice command recognition |

## Using GPT-4o for Robot Task Planning

```python
from openai import OpenAI
import base64

class RobotTaskPlanner:
    """Use GPT-4o to decompose complex tasks into robot actions."""
    
    def __init__(self):
        self.client = OpenAI()
        self.system_prompt = """You are a robot task planner.
        Given an image of the robot's environment and a task description,
        decompose the task into a sequence of primitive actions.
        
        Available primitives:
        - move_to(x, y, z): Move end-effector to position
        - grasp(object_name): Close gripper on object
        - release(): Open gripper
        - navigate_to(location): Move base to location
        - look_at(object_name): Point camera at object
        - wait(seconds): Pause execution
        - say(text): Speak to human
        
        Output JSON array of actions."""
    
    def plan(self, image_path, task):
        # Encode image
        with open(image_path, 'rb') as f:
            image_b64 = base64.b64encode(f.read()).decode()
        
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": [
                    {"type": "text", "text": f"Task: {task}"},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{image_b64}"
                    }}
                ]}
            ],
            response_format={"type": "json_object"},
            temperature=0.1,
        )
        
        return json.loads(response.choices[0].message.content)

# Usage
planner = RobotTaskPlanner()
plan = planner.plan(
    "camera_view.jpg", 
    "Make me a cup of coffee"
)
# Output: [
#   {"action": "navigate_to", "params": {"location": "kitchen_counter"}},
#   {"action": "look_at", "params": {"object_name": "coffee_machine"}},
#   {"action": "grasp", "params": {"object_name": "coffee_pod"}},
#   {"action": "move_to", "params": {"x": 0.3, "y": 0.0, "z": 0.2}},
#   {"action": "release"},
#   ...
# ]
```

## SAM 2 for Universal Object Segmentation

```python
from sam2 import SAM2ImagePredictor

class RobotObjectSegmenter:
    """Segment ANY object the robot needs to interact with."""
    
    def __init__(self):
        self.predictor = SAM2ImagePredictor.from_pretrained(
            "facebook/sam2-hiera-large"
        )
    
    def segment_object(self, image, object_description):
        """
        Segment an object described in natural language.
        Uses CLIP to find the object, then SAM2 to segment it.
        """
        # Use CLIP to find object location
        object_location = self.clip_locate(image, object_description)
        
        # Use SAM2 to segment at that location
        self.predictor.set_image(image)
        masks, scores, _ = self.predictor.predict(
            point_coords=np.array([object_location]),
            point_labels=np.array([1]),
        )
        
        # Return best mask
        best_mask = masks[scores.argmax()]
        return best_mask
```

## Whisper for Voice Commands

```python
import whisper
import sounddevice as sd
import numpy as np

class VoiceCommandNode:
    """Convert spoken commands to text for robot control."""
    
    def __init__(self):
        self.model = whisper.load_model("medium")
        self.sample_rate = 16000
    
    def listen(self, duration=5):
        """Listen for a voice command."""
        print("🎤 Listening...")
        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        
        # Transcribe
        result = self.model.transcribe(
            audio.flatten(),
            language="en",
            fp16=True,
        )
        
        command = result["text"].strip()
        print(f"📝 Heard: '{command}'")
        return command
```

## Summary

Foundation models provide the "common sense" that traditional robotics lacks. By combining vision models (CLIP, SAM2), language models (GPT-4o), and audio models (Whisper), robots can understand and interact with the world in natural, human-like ways.

---

**Next:** [LLM-Guided Robot Control →](/docs/module-4/llm-guided-control)
