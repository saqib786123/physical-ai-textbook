---
sidebar_position: 2
title: "Isaac Sim Deep Dive"
description: "Advanced Isaac Sim features: scene creation, robot control, and domain randomization."
---

# Isaac Sim Deep Dive

> *"Isaac Sim turns your GPU into a reality factory — generating millions of training scenarios in hours."*

## Scene Management with USD

**Universal Scene Description (USD)** is the scene format used by Isaac Sim. Originally developed by Pixar for film production, USD enables:

- Hierarchical scene composition (layers)
- Non-destructive editing (overrides)
- Collaboration (multiple people editing simultaneously)
- Cross-tool interoperability (Blender, Maya, 3ds Max)

```python
from pxr import Usd, UsdGeom, UsdPhysics

# Open or create a USD stage
stage = Usd.Stage.CreateNew("robot_scene.usda")

# Set meters as the unit
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Add a ground plane
ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
ground.CreatePointsAttr([(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)])

# Save
stage.GetRootLayer().Save()
```

## Domain Randomization

The most powerful feature for training robust AI:

```python
import omni.replicator.core as rep

# Randomize lighting
with rep.new_layer():
    light = rep.create.light(
        light_type="distant",
        temperature=rep.distribution.uniform(3000, 7000),
        intensity=rep.distribution.uniform(500, 2000),
    )

# Randomize object textures
with rep.new_layer():
    materials = rep.randomizer.materials(
        materials=rep.distribution.choice([
            "OmniPBR/wood_01", "OmniPBR/metal_02", "OmniPBR/plastic_01"
        ])
    )

# Randomize camera position
with rep.new_layer():
    camera = rep.create.camera()
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 1), (2, 2, 3)),
            look_at="/World/Robot"
        )
```

### Why Domain Randomization Matters

| Without DR | With DR |
|-----------|---------|
| Model only works in exact simulation lighting | Model works in any lighting |
| Fails on different colored objects | Handles color variation |
| Specific camera angle required | Works from any viewpoint |
| 😞 Fails in real world | ✅ Transfers to real world |

## Parallel Simulation

Isaac Sim can run **thousands of environments** simultaneously:

```python
from omni.isaac.lab.envs import ManagerBasedRLEnv

# Create 4096 parallel environments on GPU
env = ManagerBasedRLEnv(
    cfg=HumanoidEnvCfg(),
    num_envs=4096,  # 4096 robots training simultaneously!
)

# Each step advances ALL environments
obs = env.reset()
for i in range(1_000_000):
    actions = policy(obs)
    obs, rewards, dones, infos = env.step(actions)
```

## Synthetic Data Generation

```python
import omni.replicator.core as rep

# Setup annotators for automatic labeling
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
bbox_2d = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
semantic = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")

# Generate labeled dataset
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="synthetic_dataset/",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True,
)

# Run randomization and capture
for i in range(10000):
    rep.orchestrator.step()  # Apply randomizations
    writer.attach([rgb, bbox_2d, semantic, depth])
```

## Summary

Isaac Sim provides the most advanced simulation capabilities for Physical AI: photorealistic rendering, domain randomization for robust training, GPU-parallel environments for fast RL, and synthetic data generation for supervised learning.

---

**Next:** [Synthetic Data Generation →](/docs/module-3/synthetic-data-generation)
