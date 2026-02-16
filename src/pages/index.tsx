import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Translate, { translate } from '@docusaurus/Translate';

function HeroSection() {
  return (
    <section className="hero-section" id="hero">
      <div className="hero-content">
        <span className="hero-badge">🤖 <Translate id="hero.badge">Panaversity Textbook Series</Translate></span>
        <h1 className="hero-title">
          <span className="gradient-text"><Translate id="hero.title.part1">Physical AI</Translate></span>
          <br />
          <Translate id="hero.title.part2">& Humanoid Robotics</Translate>
        </h1>
        <p className="hero-subtitle">
          <Translate id="hero.subtitle">
            Bridging the digital brain and the physical body. Master ROS 2, Gazebo, NVIDIA Isaac,
            and Vision-Language-Action models to build the next generation of embodied intelligence.
          </Translate>
        </p>
        <div className="hero-actions">
          <Link to="/docs/intro" className="hero-btn hero-btn-primary">
            <Translate id="hero.actions.start">📖 Start Reading</Translate>
          </Link>
          <Link to="/docs/foundations/what-is-physical-ai" className="hero-btn hero-btn-secondary">
            <Translate id="hero.actions.explore">🧭 Explore Foundations</Translate>
          </Link>
        </div>
        <div className="hero-stats">
          <div className="stat-item">
            <span className="stat-number">4</span>
            <span className="stat-label"><Translate id="hero.stats.modules">Core Modules</Translate></span>
          </div>
          <div className="stat-item">
            <span className="stat-number">13</span>
            <span className="stat-label"><Translate id="hero.stats.weeks">Weeks of Learning</Translate></span>
          </div>
          <div className="stat-item">
            <span className="stat-number">30+</span>
            <span className="stat-label"><Translate id="hero.stats.chapters">In-Depth Chapters</Translate></span>
          </div>
          <div className="stat-item">
            <span className="stat-number">1</span>
            <span className="stat-label"><Translate id="hero.stats.project">Capstone Project</Translate></span>
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
      title: translate({ message: 'The Robotic Nervous System', id: 'module1.title' }),
      description: translate({ message: 'Master ROS 2 — the middleware that connects sensors, actuators, and AI. Learn nodes, topics, services, and Python bridges with rclpy.', id: 'module1.desc' }),
      topics: ['ROS 2', 'Nodes & Topics', 'rclpy', 'URDF'],
      link: '/docs/module-1/introduction-to-ros2',
    },
    {
      icon: '🌐',
      number: 'Module 2',
      title: translate({ message: 'The Digital Twin', id: 'module2.title' }),
      description: translate({ message: 'Build virtual worlds with Gazebo and Unity. Simulate physics, gravity, collisions, and sensor data before touching real hardware.', id: 'module2.desc' }),
      topics: ['Gazebo', 'Unity', 'Physics Sim', 'Sensors'],
      link: '/docs/module-2/gazebo-simulation',
    },
    {
      icon: '🧠',
      number: 'Module 3',
      title: translate({ message: 'The AI-Robot Brain', id: 'module3.title' }),
      description: translate({ message: 'Harness NVIDIA Isaac for photorealistic simulation, Visual SLAM, synthetic data, and path planning for bipedal humanoids.', id: 'module3.desc' }),
      topics: ['Isaac Sim', 'VSLAM', 'Nav2', 'Sim-to-Real'],
      link: '/docs/module-3/nvidia-isaac-platform',
    },
    {
      icon: '🗣️',
      number: 'Module 4',
      title: translate({ message: 'Vision-Language-Action', id: 'module4.title' }),
      description: translate({ message: 'Converge LLMs and Robotics. Voice commands to robot actions, cognitive planning, and multi-modal human-robot interaction.', id: 'module4.desc' }),
      topics: ['Whisper', 'LLM Planning', 'VLA Models', 'Conversational AI'],
      link: '/docs/module-4/vision-language-action',
    },
  ];

  return (
    <section className="modules-section" id="modules">
      <div className="section-header">
        <span className="section-label"><Translate id="modules.label">Course Curriculum</Translate></span>
        <h2 className="section-title"><Translate id="modules.title">Four Pillars of Physical AI</Translate></h2>
        <p className="section-subtitle">
          <Translate id="modules.subtitle">
            A structured journey from middleware mastery to autonomous humanoid robots that understand and interact with the physical world.
          </Translate>
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
      title: translate({ message: 'Human-Centered Design', id: 'why1.title' }),
      text: translate({ message: 'Humanoid robots excel in human environments because they share our physical form — doors, stairs, tools all designed for human bodies.', id: 'why1.text' }),
    },
    {
      icon: '📊',
      title: translate({ message: 'Abundant Training Data', id: 'why2.title' }),
      text: translate({ message: 'Every video of humans demonstrates tasks robots can learn. Sim-to-real transfer makes training scalable and safe.', id: 'why2.text' }),
    },
    {
      icon: '🔮',
      title: translate({ message: 'Future of Work', id: 'why3.title' }),
      text: translate({ message: 'The partnership of people, AI agents, and robots will transform industries — creating massive demand for new skills.', id: 'why3.text' }),
    },
    {
      icon: '🧪',
      title: translate({ message: 'From Digital to Physical', id: 'why4.title' }),
      text: translate({ message: 'Physical AI represents the transition from models confined to digital environments to embodied intelligence in physical space.', id: 'why4.text' }),
    },
  ];

  return (
    <section className="why-section" id="why">
      <div className="section-header">
        <span className="section-label"><Translate id="why.label">Why This Matters</Translate></span>
        <h2 className="section-title"><Translate id="why.title">The Age of Embodied Intelligence</Translate></h2>
        <p className="section-subtitle">
          <Translate id="why.subtitle">
            AI is moving beyond screens and into the real world. This textbook prepares you for that revolution.
          </Translate>
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
        <span className="section-label"><Translate id="cta.label">Ready to Build the Future?</Translate></span>
        <h2 className="section-title" style={{ marginTop: '1rem' }}>
          <Translate id="cta.title">Start Your Physical AI Journey</Translate>
        </h2>
        <p className="section-subtitle" style={{ margin: '1rem auto 2rem' }}>
          <Translate id="cta.subtitle">
            From ROS 2 fundamentals to building an autonomous humanoid that can hear, see, think, and act —
            your comprehensive guide awaits.
          </Translate>
        </p>
        <div className="hero-actions">
          <Link to="/docs/intro" className="hero-btn hero-btn-primary">
            <Translate id="cta.actions.begin">📘 Begin Chapter 1</Translate>
          </Link>
          <Link to="/docs/setup-your-lab" className="hero-btn hero-btn-secondary">
            <Translate id="cta.actions.setup">🔧 Setup Your Lab</Translate>
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
