---
sidebar_position: 3
title: "Synthetic Data Generation"
description: "Generating labeled training data using simulation for training perception models."
---

# Synthetic Data Generation

> *"Why label data manually when simulation can generate millions of perfectly labeled images automatically?"*

## The Data Problem

Training perception models requires massive labeled datasets. Traditional approach: hire annotators to label thousands of images. This is slow, expensive, and error-prone.

**Solution:** Generate unlimited, perfectly labeled data in simulation.

### Synthetic vs Real Data

| Aspect | Real Data | Synthetic Data |
|--------|-----------|----------------|
| **Cost per image** | $0.05-0.50 (labeling) | ~$0.0001 (compute) |
| **Label accuracy** | ~95% (human error) | 100% (automatic) |
| **Volume** | Thousands per week | Millions per hour |
| **Diversity** | Limited by collection | Unlimited (randomization) |
| **Edge cases** | Rare and dangerous | Easy to generate |
| **Privacy** | Consent required | No privacy concerns |

## Generating Data with Replicator

```python
import omni.replicator.core as rep

def generate_object_detection_dataset():
    """Generate labeled images for object detection."""
    
    # Create basic scene
    rep.create.plane(scale=10, material=rep.create.material_omnipbr(
        diffuse_texture=rep.distribution.choice([
            "textures/wood_floor.png",
            "textures/tile_floor.png",
            "textures/carpet.png"
        ])
    ))
    
    # Create randomized objects
    cups = rep.create.from_usd(
        "assets/cup.usd",
        count=rep.distribution.uniform(1, 5),
    )
    
    # Randomize positions
    with cups:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 1.5)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        )
    
    # Setup camera
    camera = rep.create.camera(position=(0, -3, 2), look_at=(0, 0, 0.5))
    
    # Setup writer
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="datasets/cup_detection/",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
    )
    
    # Render dataset
    render_product = rep.create.render_product(camera, (640, 480))
    writer.attach([render_product])
    
    # Generate 10,000 unique images
    rep.orchestrator.run_until_complete(num_frames=10000)
```

## Types of Annotations

Isaac Sim Replicator can automatically generate:

| Annotation Type | Description | Use Case |
|----------------|-------------|----------|
| **Bounding Box 2D** | Rectangle around objects | Object detection (YOLO) |
| **Bounding Box 3D** | 3D cuboid around objects | 3D object detection |
| **Semantic Segmentation** | Per-pixel class labels | Scene understanding |
| **Instance Segmentation** | Per-pixel instance IDs | Object counting |
| **Depth Map** | Per-pixel distance | 3D reconstruction |
| **Surface Normals** | Per-pixel surface direction | Grasp planning |
| **Skeleton** | Joint positions | Human pose estimation |
| **Optical Flow** | Per-pixel motion vectors | Motion prediction |

## Training a Model on Synthetic Data

```python
import torch
from torch.utils.data import DataLoader
from torchvision.models.detection import fasterrcnn_resnet50_fpn

# Load synthetic dataset
dataset = SyntheticObjectDataset("datasets/cup_detection/")
dataloader = DataLoader(dataset, batch_size=16, shuffle=True)

# Initialize model
model = fasterrcnn_resnet50_fpn(pretrained=True, num_classes=5)

# Train on synthetic data
optimizer = torch.optim.SGD(model.parameters(), lr=0.005, momentum=0.9)

for epoch in range(50):
    for images, targets in dataloader:
        loss_dict = model(images, targets)
        losses = sum(loss for loss in loss_dict.values())
        
        optimizer.zero_grad()
        losses.backward()
        optimizer.step()
    
    print(f"Epoch {epoch}: Loss = {losses.item():.4f}")

# Save model for deployment to Jetson
torch.save(model.state_dict(), "cup_detector_synthetic.pth")
```

## Summary

Synthetic data generation eliminates the data bottleneck in training perception models. Using Isaac Sim's Replicator, you can generate millions of perfectly labeled images with automatic domain randomization.

---

**Next:** [Isaac ROS Perception →](/docs/module-3/isaac-ros-perception)
