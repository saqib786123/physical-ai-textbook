import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

function HeroSection() {
  return (
    <section className="hero-section" id="hero">
      <div className="hero-content">
        <span className="hero-badge">🤖 Panaversity Textbook Series</span>
        <h1 className="hero-title">
          <span className="gradient-text">Physical AI</span>
          <br />
          & Humanoid Robotics
        </h1>
        <p className="hero-subtitle">
          Bridging the digital brain and the physical body. Master ROS 2, Gazebo, NVIDIA Isaac,
          and Vision-Language-Action models to build the next generation of embodied intelligence.
        </p>
        <div className="hero-actions">
          <Link to="/docs/intro" className="hero-btn hero-btn-primary">
            📖 Start Reading
          </Link>
          <Link to="/docs/foundations/what-is-physical-ai" className="hero-btn hero-btn-secondary">
            🧭 Explore Foundations
          </Link>
        </div>
        <div className="hero-stats">
          <div className="stat-item">
            <span className="stat-number">4</span>
            <span className="stat-label">Core Modules</span>
          </div>
          <div className="stat-item">
            <span className="stat-number">13</span>
            <span className="stat-label">Weeks of Learning</span>
          </div>
          <div className="stat-item">
            <span className="stat-number">30+</span>
            <span className="stat-label">In-Depth Chapters</span>
          </div>
          <div className="stat-item">
            <span className="stat-number">1</span>
            <span className="stat-label">Capstone Project</span>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      icon: '🧬',
      number: 'Module 1',
      title: 'The Robotic Nervous System',
      description: 'Master ROS 2 — the middleware that connects sensors, actuators, and AI. Learn nodes, topics, services, and Python bridges with rclpy.',
      topics: ['ROS 2', 'Nodes & Topics', 'rclpy', 'URDF'],
      link: '/docs/module-1/introduction-to-ros2',
    },
    {
      icon: '🌐',
      number: 'Module 2',
      title: 'The Digital Twin',
      description: 'Build virtual worlds with Gazebo and Unity. Simulate physics, gravity, collisions, and sensor data before touching real hardware.',
      topics: ['Gazebo', 'Unity', 'Physics Sim', 'Sensors'],
      link: '/docs/module-2/gazebo-simulation',
    },
    {
      icon: '🧠',
      number: 'Module 3',
      title: 'The AI-Robot Brain',
      description: 'Harness NVIDIA Isaac for photorealistic simulation, Visual SLAM, synthetic data, and path planning for bipedal humanoids.',
      topics: ['Isaac Sim', 'VSLAM', 'Nav2', 'Sim-to-Real'],
      link: '/docs/module-3/nvidia-isaac-platform',
    },
    {
      icon: '🗣️',
      number: 'Module 4',
      title: 'Vision-Language-Action',
      description: 'Converge LLMs and Robotics. Voice commands to robot actions, cognitive planning, and multi-modal human-robot interaction.',
      topics: ['Whisper', 'LLM Planning', 'VLA Models', 'Conversational AI'],
      link: '/docs/module-4/vision-language-action',
    },
  ];

  return (
    <section className="modules-section" id="modules">
      <div className="section-header">
        <span className="section-label">Course Curriculum</span>
        <h2 className="section-title">Four Pillars of Physical AI</h2>
        <p className="section-subtitle">
          A structured journey from middleware mastery to autonomous humanoid robots that understand and interact with the physical world.
        </p>
      </div>
      <div className="modules-grid">
        {modules.map((mod, idx) => (
          <Link key={idx} to={mod.link} className="module-card">
            <span className="module-icon">{mod.icon}</span>
            <span className="module-number">{mod.number}</span>
            <h3 className="module-card-title">{mod.title}</h3>
            <p className="module-card-description">{mod.description}</p>
            <div className="module-topics">
              {mod.topics.map((topic, i) => (
                <span key={i} className="module-topic-tag">{topic}</span>
              ))}
            </div>
          </Link>
        ))}
      </div>
    </section>
  );
}

function WhySection() {
  const reasons = [
    {
      icon: '🏭',
      title: 'Human-Centered Design',
      text: 'Humanoid robots excel in human environments because they share our physical form — doors, stairs, tools all designed for human bodies.',
    },
    {
      icon: '📊',
      title: 'Abundant Training Data',
      text: 'Every video of humans demonstrates tasks robots can learn. Sim-to-real transfer makes training scalable and safe.',
    },
    {
      icon: '🔮',
      title: 'Future of Work',
      text: 'The partnership of people, AI agents, and robots will transform industries — creating massive demand for new skills.',
    },
    {
      icon: '🧪',
      title: 'From Digital to Physical',
      text: 'Physical AI represents the transition from models confined to digital environments to embodied intelligence in physical space.',
    },
  ];

  return (
    <section className="why-section" id="why">
      <div className="section-header">
        <span className="section-label">Why This Matters</span>
        <h2 className="section-title">The Age of Embodied Intelligence</h2>
        <p className="section-subtitle">
          AI is moving beyond screens and into the real world. This textbook prepares you for that revolution.
        </p>
      </div>
      <div className="why-grid">
        {reasons.map((reason, idx) => (
          <div key={idx} className="why-card">
            <span className="why-icon">{reason.icon}</span>
            <h3 className="why-card-title">{reason.title}</h3>
            <p className="why-card-text">{reason.text}</p>
          </div>
        ))}
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className="cta-section" id="cta">
      <div className="cta-content">
        <span className="section-label">Ready to Build the Future?</span>
        <h2 className="section-title" style={{ marginTop: '1rem' }}>
          Start Your Physical AI Journey
        </h2>
        <p className="section-subtitle" style={{ margin: '1rem auto 2rem' }}>
          From ROS 2 fundamentals to building an autonomous humanoid that can hear, see, think, and act —
          your comprehensive guide awaits.
        </p>
        <div className="hero-actions">
          <Link to="/docs/intro" className="hero-btn hero-btn-primary">
            📘 Begin Chapter 1
          </Link>
          <Link to="/docs/appendices/lab-setup-guide" className="hero-btn hero-btn-secondary">
            🔧 Setup Your Lab
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive textbook for Physical AI & Humanoid Robotics — covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models. By Panaversity."
    >
      <HeroSection />
      <ModulesSection />
      <WhySection />
      <CTASection />
    </Layout>
  );
}
