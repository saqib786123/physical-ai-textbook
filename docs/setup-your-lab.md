---
sidebar_position: 2
title: "🛠️ Setup Your Lab"
description: "Step-by-step guide to preparing your Physical AI development environment."
---

# Setup Your Lab: Environment Preparation

Before we dive into building humanoid robots, we need to ensure your workstation is equipped with the right tools. Follow this guide carefully to set up your Physical AI development environment.

---

## 1. System Requirements
- **OS**: Ubuntu 22.04 LTS (Recommended) or Windows 11 with WSL2.
- **GPU**: NVIDIA RTX 3060 or higher (Required for NVIDIA Isaac Sim).
- **RAM**: 16GB Minimum (32GB Recommended).
- **Disk**: 100GB Free Space.

---

## 2. Core Programming Tools

### Install Node.js
We use Node.js for the AI Backend and Docusaurus Frontend.
```bash
# Using NVM (Recommended)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.5/install.sh | bash
nvm install 20
```

### Install Python
Python is the primary language for robotics and AI.
```bash
sudo apt update
sudo apt install python3-pip python3-dev
```

---

## 3. Robotics Frameworks

### Install ROS 2 (Humble Hawksbill)
ROS 2 is the "nervous system" of our robot.
```bash
# Follow the official ROS 2 Humble installation for your OS
# Link: https://docs.ros.org/en/humble/Installation.html
```

### Install Gazebo Sim
Gazebo is our primary physics simulator.
```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-gz
```

---

## 4. NVIDIA Isaac Sim
Advanced photorealistic simulation and synthetic data generation.
1. Download **NVIDIA Omniverse Launcher**.
2. Install **Isaac Sim** from the Exchange tab.
3. Configure the **Isaac ROS** workspace.

---

## 5. Setting Up This Project

Clone the textbook repository and install dependencies:

```bash
git clone https://github.com/saqib786123/physical-ai-textbook.git
cd physical-ai-textbook

# Install Frontend Dependencies
npm install

# Setup Backend
cd backend
npm install
cp .env.example .env
```

---

## 6. Verification: Running a Test
To ensure everything is working correctly, run the following:

### Start the AI Backend
```bash
node backend/server.js
```

### Start the Simulator
```bash
# Launch a basic ROS 2 topic test
ros2 run demo_nodes_cpp talker
```

### Launch the Textbook site
```bash
npm start
```

---

## 7. System Readiness Checklist
- [ ] `node -v` returns v20+
- [ ] `ros2 doctor` passes
- [ ] NVIDIA drivers are up to date (`nvidia-smi`)
- [ ] Backend server is running at `localhost:3001`

**Congratulations! Your lab is ready. Let's start building Physical AI.**
