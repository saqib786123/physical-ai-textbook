import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '📘 Introduction to Physical AI',
    },
    {
      type: 'category',
      label: '🌍 Foundations of Physical AI',
      collapsed: false,
      items: [
        'foundations/what-is-physical-ai',
        'foundations/embodied-intelligence',
        'foundations/humanoid-robotics-landscape',
        'foundations/sensor-systems',
        'foundations/hardware-requirements',
      ],
    },
    {
      type: 'category',
      label: '🧬 Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: true,
      items: [
        'module-1/introduction-to-ros2',
        'module-1/ros2-architecture',
        'module-1/nodes-topics-services',
        'module-1/building-ros2-packages',
        'module-1/launch-files-parameters',
        'module-1/rclpy-python-bridge',
        'module-1/urdf-humanoid-description',
      ],
    },
    {
      type: 'category',
      label: '🌐 Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: true,
      items: [
        'module-2/gazebo-simulation',
        'module-2/urdf-sdf-formats',
        'module-2/physics-simulation',
        'module-2/sensor-simulation',
        'module-2/unity-visualization',
        'module-2/human-robot-interaction-sim',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: true,
      items: [
        'module-3/nvidia-isaac-platform',
        'module-3/isaac-sim-overview',
        'module-3/synthetic-data-generation',
        'module-3/isaac-ros-perception',
        'module-3/visual-slam',
        'module-3/nav2-path-planning',
        'module-3/reinforcement-learning',
        'module-3/sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: '🔮 Module 4: Vision-Language-Action (VLA)',
      collapsed: true,
      items: [
        'module-4/vision-language-action',
        'module-4/multimodal-foundation-models',
        'module-4/llm-guided-control',
        'module-4/end-to-end-learning',
      ],
    },
    {
      type: 'category',
      label: '🦾 Humanoid Development',
      collapsed: true,
      items: [
        'humanoid-development/bipedal-locomotion',
        'humanoid-development/dexterous-manipulation',
        'humanoid-development/whole-body-control',
      ],
    },
    {
      type: 'category',
      label: '🎯 Capstone Project',
      collapsed: true,
      items: [
        'capstone/capstone-overview',
      ],
    },
    {
      type: 'category',
      label: '📎 Appendices',
      collapsed: true,
      items: [
        'appendices/glossary',
        'appendices/resources',
      ],
    },
  ],
};

export default sidebars;
