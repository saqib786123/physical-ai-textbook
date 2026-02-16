---
sidebar_position: 5
title: "Hardware Requirements"
description: "Complete hardware setup guide for the Physical AI course, including workstation, edge computing, and robot lab options."
keywords: [hardware requirements, NVIDIA GPU, Jetson, RealSense, robot hardware, lab setup]
---

# Hardware Requirements

> *"Building a Physical AI lab is a significant investment. Choose between high CapEx (on-premise) or high OpEx (cloud-native)."*

## Overview

This course sits at the intersection of three heavy computational loads:

1. **Physics Simulation** (Isaac Sim / Gazebo)
2. **Visual Perception** (SLAM / Computer Vision)
3. **Generative AI** (LLMs / VLA models)

This chapter helps you choose the right hardware for your budget and learning goals.

## Tier 1: The "Digital Twin" Workstation

This is the **most critical** component. NVIDIA Isaac Sim is an Omniverse application that requires RTX (Ray Tracing) capabilities.

:::danger Standard laptops won't work
MacBooks and non-RTX Windows machines **cannot** run Isaac Sim. You need a dedicated NVIDIA RTX GPU.
:::

### Minimum Specifications

| Component | Minimum | Recommended | Ideal |
|-----------|---------|-------------|-------|
| **GPU** | RTX 3060 (12GB) | RTX 4070 Ti (12GB) | RTX 4090 (24GB) |
| **CPU** | Intel i5 12th Gen | Intel i7 13th Gen+ | AMD Ryzen 9 |
| **RAM** | 32 GB DDR4 | 64 GB DDR5 | 128 GB DDR5 |
| **Storage** | 512 GB NVMe SSD | 1 TB NVMe SSD | 2 TB NVMe SSD |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

### Why These Specs?

- **GPU (The Bottleneck):** You need high VRAM to load USD (Universal Scene Description) assets for the robot and environment, plus run VLA models simultaneously
- **CPU:** Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive
- **RAM:** 32 GB is the absolute minimum but will crash during complex scene rendering
- **OS:** While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux

:::tip Dual-Boot
If you have a Windows machine with an RTX GPU, set up Ubuntu 22.04 as a dual-boot or use WSL2 (Windows Subsystem for Linux) for ROS 2 development.
:::

## Tier 2: The "Physical AI" Edge Kit

Since a full humanoid robot is expensive, students learn Physical AI by setting up the "nervous system" on a desk before deploying it to a robot.

### The Economy Jetson Student Kit (~$700)

| Component | Model | Price | Notes |
|-----------|-------|-------|-------|
| **The Brain** | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 | 40 TOPS AI performance |
| **The Eyes** | Intel RealSense D435i | $349 | RGB + Depth + IMU (buy the "i" version!) |
| **The Ears** | ReSpeaker USB Mic Array v2.0 | $69 | Far-field microphone array |
| **Wi-Fi** | (Included in Dev Kit) | $0 | Pre-installed in the "Super" kit |
| **Power/Misc** | SD Card (128GB) + cables | $30 | High-endurance microSD required |
| **TOTAL** | | **~$700** | |

### Jetson Orin Nano: Key Specs

```
┌────────────────────────────────────────────┐
│  NVIDIA Jetson Orin Nano Super Dev Kit     │
├────────────────────────────────────────────┤
│  AI Performance:     40 TOPS               │
│  GPU:                1024-core NVIDIA Ampere│
│  CPU:                6-core Arm A78AE       │
│  Memory:             8 GB LPDDR5            │
│  Storage:            microSD slot           │
│  Interfaces:         USB 3.2, PCIe, CSI     │
│  Power:              7-15W                  │
│  Size:               100 x 79 mm           │
│  OS:                 JetPack (Ubuntu-based)  │
└────────────────────────────────────────────┘
```

## Tier 3: Robot Lab Options

### Option A: The "Proxy" Approach (Budget: ~$3,000)

Use a quadruped or robotic arm as a proxy. 90% of software principles transfer to humanoids.

**Recommended:** Unitree Go2 Edu (~$1,800 - $3,000)
- ✅ Highly durable
- ✅ Excellent ROS 2 support
- ✅ Multiple units affordable
- ❌ Not a biped (humanoid)

### Option B: The "Miniature Humanoid" (~$600 - $16,000)

| Robot | Price | Pros | Cons |
|-------|-------|------|------|
| Hiwonder TonyPi Pro | ~$600 | Affordable, walking demos | Raspberry Pi - can't run Isaac ROS |
| Unitree G1 | ~$16,000 | Dynamic walking, open SDK | Expensive for students |
| Robotis OP3 | ~$12,000 | Stable platform, research-ready | Older design |

### Option C: Premium Lab (Sim-to-Real)

**Unitree G1 Humanoid**: One of the few commercially available humanoids with an SDK open enough for students to inject their own ROS 2 controllers.

## Cloud-Native Lab (Alternative)

Don't have an RTX GPU? Use cloud instances instead.

### AWS Cloud Setup

| Resource | Type | Cost |
|----------|------|------|
| GPU Instance | g5.2xlarge (A10G GPU, 24GB) | ~$1.50/hour |
| Storage | EBS (environment saves) | ~$25/quarter |
| **Total** (120 hours) | | **~$205/quarter** |

```bash
# Launch an Isaac Sim instance on AWS
aws ec2 run-instances \
  --image-id ami-0xxxxxxxx \
  --instance-type g5.2xlarge \
  --key-name your-key \
  --security-groups IsaacSim-SG
```

:::warning The Latency Trap
Simulating in the cloud works for training. But controlling a real robot from a cloud instance is **dangerous due to latency**. 

**Solution:** Train in the cloud → Download model weights → Flash to local Jetson.
:::

## Architecture Summary

```
┌─────────────────┬──────────────────────┬─────────────────────┐
│    Component     │      Hardware        │      Function       │
├─────────────────┼──────────────────────┼─────────────────────┤
│ Sim Rig         │ PC + RTX 4080        │ Isaac Sim, Gazebo,  │
│                 │ + Ubuntu 22.04       │ Unity, LLM training │
├─────────────────┼──────────────────────┼─────────────────────┤
│ Edge Brain      │ Jetson Orin Nano     │ Runs inference,     │
│                 │                      │ deploys ROS 2 nodes │
├─────────────────┼──────────────────────┼─────────────────────┤
│ Sensors         │ RealSense + LiDAR    │ Feed real-world     │
│                 │                      │ data to AI          │
├─────────────────┼──────────────────────┼─────────────────────┤
│ Actuator        │ Unitree Go2/G1       │ Receives motor      │
│                 │ (Shared)             │ commands from Jetson │
└─────────────────┴──────────────────────┴─────────────────────┘
```

## Recommendation by Budget

| Budget | What To Buy | What You Can Do |
|--------|------------|-----------------|
| **$0** | Use simulation only (Gazebo on any Linux PC) | Complete Modules 1-2 |
| **$250** | Jetson Orin Nano only | Edge AI deployment |
| **$700** | Full Jetson Student Kit | Modules 1-4 (edge) |
| **$3,000** | Kit + Unitree Go2 | Full course with real robot |
| **$5,000** | RTX Workstation + Jetson Kit | Isaac Sim + Edge deployment |
| **$20,000+** | Everything + Unitree G1 | Full course with humanoid |

## Summary

The right hardware depends on your goals and budget. The most important thing is **an NVIDIA GPU for simulation** — everything else can be done in software first and deployed to hardware later.

---

**Next:** [Module 1: Introduction to ROS 2 →](/docs/module-1/introduction-to-ros2)
