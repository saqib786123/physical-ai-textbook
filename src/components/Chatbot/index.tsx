import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { useAuth } from '../../context/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Type definition for chat messages
type Message = {
  id: string;
  sender: 'user' | 'bot';
  text: string;
};

// Fallback knowledge base in case API is down/rate-limited
const KNOWLEDGE_BASE_FALLBACK = {
  default: "I'm having trouble connecting to my brain (OpenAI API). But I can tell you that **ROS 2** is the middleware for robots and **Gazebo** is a simulator!",
  ros: "ROS 2 (Robot Operating System) is the middleware that connects sensors, actuators, and AI. It uses a graph architecture with nodes, topics, and services.",
  gazebo: "Gazebo is a 3D simulator that allows you to simulate physics, gravity, collisions, and sensor data before touching real hardware.",
  isaac: "NVIDIA Isaac Sim is a photorealistic simulation platform powered by Omniverse, used for synthetic data generation and reinforcement learning.",
};

export default function Chatbot() {
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl || 'http://localhost:3001';
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    { id: '1', sender: 'bot', text: 'Hello! I\'m your Physical AI assistant. Ask me about ROS 2, Gazebo, or Isaac Sim.' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Personalized greeting when user logs in
  useEffect(() => {
    if (user && messages.length === 1) {
      setMessages([{
        id: '1',
        sender: 'bot',
        text: `Welcome back, **${user.username}**! I'm ready to help you with your Physical AI studies. What's on your mind today?`
      }]);
    }
  }, [user]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  const fallbackAnswer = (query: string): string => {
    const lowerQuery = query.toLowerCase();
    if (lowerQuery.includes('ros')) return KNOWLEDGE_BASE_FALLBACK.ros;
    if (lowerQuery.includes('gazebo')) return KNOWLEDGE_BASE_FALLBACK.gazebo;
    if (lowerQuery.includes('isaac')) return KNOWLEDGE_BASE_FALLBACK.isaac;
    return KNOWLEDGE_BASE_FALLBACK.default;
  };

  const handleSend = async () => {
    if (!inputValue.trim()) return;

    const userText = inputValue;
    const userMsg: Message = {
      id: Date.now().toString(),
      sender: 'user',
      text: userText
    };

    setMessages(prev => [...prev, userMsg]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Attempt to call the standalone RAG API
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userText,
          username: user?.username || 'Guest'
        }),
      });

      if (!response.ok) {
        throw new Error('API Error');
      }

      const data = await response.json();
      const botMsg: Message = {
        id: (Date.now() + 1).toString(),
        sender: 'bot',
        text: data.answer || "I received an empty response."
      };
      setMessages(prev => [...prev, botMsg]);

    } catch (error) {
      console.warn("RAG API failed, using fallback:", error);
      // Fallback to local knowledge if API fails
      const botMsg: Message = {
        id: (Date.now() + 1).toString(),
        sender: 'bot',
        text: fallbackAnswer(userText) + (user ? ` (Personalized Offline Mode for ${user.username})` : " (Offline Mode)")
      };
      setMessages(prev => [...prev, botMsg]);
    } finally {
      setIsLoading(false);
    }
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
                <div dangerouslySetInnerHTML={{ __html: msg.text.replace(/\n/g, '<br/>') }} />
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
          <svg viewBox="0 0 24 24"><path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" /></svg>
        ) : (
          <svg viewBox="0 0 24 24"><path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" /></svg>
        )}
      </button>
    </div>
  );
}
