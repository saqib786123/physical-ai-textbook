---
sidebar_position: 4
title: "Sensor Simulation"
description: "Simulating cameras, LiDAR, IMUs, and other sensors in Gazebo for ROS 2."
keywords: [sensor simulation, virtual sensors, Gazebo sensors, camera simulation, LiDAR simulation]
---

# Sensor Simulation

> *"Virtual sensors generate the same data as real sensors — letting you develop perception algorithms without physical hardware."*

## Why Simulate Sensors?

Simulated sensors produce data in the exact same ROS 2 message formats as real sensors. Code developed with simulated sensors works with real sensors **without modification**.

## Camera Simulation

### Adding a Camera to Your Robot (SDF Plugin)

```xml
<!-- Camera sensor in Gazebo -->
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin filename="gz-sim-sensors-system"
          name="gz::sim::systems::Sensors">
  </plugin>
</sensor>
```

### Depth Camera

```xml
<sensor name="depth_camera" type="depth_camera">
  <always_on>true</always_on>
  <update_rate>15</update_rate>
  <camera>
    <horizontal_fov>1.2</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
  </camera>
</sensor>
```

## LiDAR Simulation

```xml
<sensor name="lidar" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.01</stddev>     
    </noise>
  </lidar>
</sensor>
```

## IMU Simulation

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian">
        <mean>0</mean><stddev>0.01</stddev>
      </noise></x>
    </angular_velocity>
    <linear_acceleration>
      <z><noise type="gaussian">
        <mean>0</mean><stddev>0.1</stddev>
      </noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Bridging Simulated Sensors to ROS 2

```bash
# Bridge all sensor topics
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

## Adding Sensor Noise

Real sensors are **noisy**. Simulating noise makes your algorithms robust:

| Sensor | Noise Type | Typical Values |
|--------|-----------|----------------|
| Camera | Gaussian pixel noise | σ = 0.005-0.01 |
| Depth Camera | Gaussian + quantization | σ = 1-5mm |
| LiDAR | Gaussian range noise | σ = 0.01-0.05m |
| IMU (Gyro) | Gaussian + bias drift | σ = 0.01 rad/s |
| IMU (Accel) | Gaussian + bias drift | σ = 0.1 m/s² |

## Summary

Sensor simulation enables full development of perception algorithms without physical hardware. The key is adding realistic noise models so algorithms remain robust when deployed to real sensors.

---

**Next:** [Unity Visualization →](/docs/module-2/unity-visualization)
