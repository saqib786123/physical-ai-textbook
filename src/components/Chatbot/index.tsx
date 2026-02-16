import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { useColorMode } from '@docusaurus/theme-common';

type Message = {
  id: string;
  sender: 'user' | 'bot';
  text: string;
};

const KNOWLEDGE_BASE = {
  ros: "ROS 2 (Robot Operating System) is the middleware that connects sensors, actuators, and AI. It uses a graph architecture with nodes, topics, and services.",
  gazebo: "Gazebo is a 3D simulator that allows you to simulate physics, gravity, collisions, and sensor data before touching real hardware.",
  isaac: "NVIDIA Isaac Sim is a photorealistic simulation platform powered by Omniverse, used for synthetic data generation and reinforcement learning.",
  vla: "Vision-Language-Action (VLA) models combine computer vision and large language models to enable robots to understand and act on natural language commands.",
  urdf: "URDF (Unified Robot Description Format) is an XML format used to describe the kinematic and dynamic properties of a robot."
};

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    { id: '1', sender: 'bot', text: 'Hello! I\'m your Physical AI assistant. Ask me about ROS 2, Gazebo, or Isaac Sim.' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { colorMode } = useColorMode();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  const findAnswer = (query: string): string => {
    const lowerQuery = query.toLowerCase();
    
    // Simple mock RAG search
    if (lowerQuery.includes('ros') || lowerQuery.includes('node') || lowerQuery.includes('topic')) return KNOWLEDGE_BASE.ros;
    if (lowerQuery.includes('gazebo') || lowerQuery.includes('sim')) return KNOWLEDGE_BASE.gazebo;
    if (lowerQuery.includes('isaac') || lowerQuery.includes('nvidia')) return KNOWLEDGE_BASE.isaac;
    if (lowerQuery.includes('vla') || lowerQuery.includes('vision')) return KNOWLEDGE_BASE.vla;
    if (lowerQuery.includes('urdf') || lowerQuery.includes('xml')) return KNOWLEDGE_BASE.urdf;
    
    return "I'm still learning! I can only answer questions about the core modules for now. Try asking about ROS 2 or Gazebo.";
  };

  const handleSend = async () => {
    if (!inputValue.trim()) return;

    const userMsg: Message = {
      id: Date.now().toString(),
      sender: 'user',
      text: inputValue
    };

    setMessages(prev => [...prev, userMsg]);
    setInputValue('');
    setIsLoading(true);

    // Simulate API delay
    setTimeout(() => {
      const answer = findAnswer(userMsg.text);
      const botMsg: Message = {
        id: (Date.now() + 1).toString(),
        sender: 'bot',
        text: answer
      };
      setMessages(prev => [...prev, botMsg]);
      setIsLoading(false);
    }, 1000);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') handleSend();
  };

  return (
    <div className={styles.chatbotContainer}>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>🤖 AI Assistant</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeButton}>×</button>
          </div>
          
          <div className={styles.messagesContainer}>
            {messages.map((msg) => (
              <div key={msg.id} className={`${styles.message} ${msg.sender === 'user' ? styles.userMessage : styles.botMessage}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.loadingDots}>
                  <div className={styles.dot}></div>
                  <div className={styles.dot}></div>
                  <div className={styles.dot}></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputArea}>
            <input
              type="text"
              className={styles.inputField}
              placeholder="Ask a question..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            <button onClick={handleSend} className={styles.sendButton} disabled={isLoading || !inputValue.trim()}>
              ➤
            </button>
          </div>
        </div>
      )}

      <button 
        className={styles.toggleButton} 
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle Chatbot"
      >
        {isOpen ? (
          <svg viewBox="0 0 24 24"><path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/></svg>
        ) : (
          <svg viewBox="0 0 24 24"><path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z"/></svg>
        )}
      </button>
    </div>
  );
}
