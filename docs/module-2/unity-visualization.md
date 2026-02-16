---
sidebar_position: 5
title: "Unity for Robot Visualization"
description: "Using Unity for high-fidelity robot visualization and human-robot interaction simulation."
keywords: [Unity, robot visualization, 3D rendering, HRI simulation, Unity ROS]
---

# Unity for Robot Visualization

> *"Unity provides photorealistic rendering that makes your simulation indistinguishable from reality."*

## Why Unity?

While Gazebo excels at physics simulation, **Unity** provides:

- **Photorealistic rendering** with real-time ray tracing
- **Rich environment creation** with the Unity Asset Store
- **VR/AR support** for immersive robot programming
- **Advanced animation** for realistic humanoid movements
- **Cross-platform deployment** (PC, mobile, web)

## Unity Robotics Hub

Unity provides the **Unity Robotics Hub** package for ROS 2 integration:

```
┌──────────────┐        TCP/ROS2       ┌──────────────┐
│    Unity      │◄─────────────────────►│   ROS 2      │
│               │                       │              │
│ • Rendering   │   Sensor data ──────▶ │ • Navigation │
│ • Animation   │   ◀────── Commands    │ • Planning   │
│ • UI/UX       │                       │ • Control    │
│ • VR/AR       │                       │ • AI         │
└──────────────┘                       └──────────────┘
```

### Setting Up Unity-ROS 2 Connection

```csharp
// Unity C# script for ROS 2 communication
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", OnCmdVelReceived);
    }
    
    void OnCmdVelReceived(TwistMsg msg)
    {
        // Apply velocity commands to the Unity robot model
        float linearX = (float)msg.linear.x;
        float angularZ = (float)msg.angular.z;
        
        transform.Translate(Vector3.forward * linearX * Time.deltaTime);
        transform.Rotate(Vector3.up * angularZ * Mathf.Rad2Deg * Time.deltaTime);
    }
}
```

## Creating Realistic Environments

### Importing Robot Models

```csharp
// Import URDF into Unity
using Unity.Robotics.UrdfImporter;

public class RobotImporter : MonoBehaviour
{
    void ImportRobot()
    {
        // Load URDF file
        string urdfPath = "Assets/Robots/humanoid.urdf";
        
        // Import options
        var settings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.vHACD,
        };
        
        // Create robot in scene
        UrdfRobotExtensions.Create(urdfPath, settings);
    }
}
```

### Environment Design Tips

| Element | Purpose | Implementation |
|---------|---------|----------------|
| **Lighting** | Realistic shadows, reflections | HDRP + area lights |
| **Materials** | Surface properties (PBR) | Albedo, normal, roughness maps |
| **Post-Processing** | Bloom, color grading | Unity Volume profiles |
| **Particles** | Dust, smoke, sparks | VFX Graph |
| **Audio** | Robot sounds, environment | Spatial audio sources |

## Unity vs Gazebo: When to Use Each

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Physics accuracy** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Visual quality** | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **ROS integration** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **VR/AR support** | ❌ | ⭐⭐⭐⭐⭐ |
| **Asset ecosystem** | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Best for** | Algorithm development | Visualization, HRI, demos |

## Summary

Unity complements Gazebo by providing high-fidelity visualization and rendering capabilities. Use Gazebo for physics-accurate simulation and algorithm development; use Unity for visualization, demonstrations, and human-robot interaction testing.

---

**Next:** [Human-Robot Interaction Simulation →](/docs/module-2/human-robot-interaction-sim)
