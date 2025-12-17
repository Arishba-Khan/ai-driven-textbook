import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface Source {
  title: string;
  section: string;
  page: number | string;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
}

const generateSessionId = () => {
  return 'session-' + Math.random().toString(36).substr(2, 9);
};

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionId] = useState(generateSessionId);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      role: 'assistant',
      content: 'Hello! I am your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today?'
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (e?: React.FormEvent) => {
    e?.preventDefault();
    
    if (!inputValue.trim() || isLoading) return;

    const userMessageContent = inputValue.trim();
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: userMessageContent
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    // Initial bot message placeholder
    const botMessageId = (Date.now() + 1).toString();
    setMessages(prev => [...prev, {
      id: botMessageId,
      role: 'assistant',
      content: ''
    }]);

    try {
      const response = await fetch('http://localhost:8000/chat/stream', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessageContent,
          session_id: sessionId
        }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) throw new Error('No reader available');

      let botContent = '';
      let botSources: Source[] = [];

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const dataStr = line.slice(6);
            if (dataStr === '[DONE]') continue;

            try {
              const data = JSON.parse(dataStr);

              if (data.token) {
                botContent += data.token;
                setMessages(prev => prev.map(msg => 
                  msg.id === botMessageId 
                    ? { ...msg, content: botContent } 
                    : msg
                ));
              } else if (data.sources) {
                botSources = data.sources;
                setMessages(prev => prev.map(msg => 
                  msg.id === botMessageId 
                    ? { ...msg, sources: botSources } 
                    : msg
                ));
              } else if (data.error) {
                console.error("Stream error:", data.error);
                botContent += `\n\n[Error: ${data.error.message}]`;
                 setMessages(prev => prev.map(msg => 
                  msg.id === botMessageId 
                    ? { ...msg, content: botContent } 
                    : msg
                ));
              }

            } catch (e) {
              console.error('Error parsing SSE data', e);
            }
          }
        }
      }

    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => prev.map(msg => 
        msg.id === botMessageId 
          ? { ...msg, content: "Sorry, I'm having trouble connecting to the backend. Please check if the server is running." } 
          : msg
      ));
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
            <button className={styles.closeButton} onClick={toggleChat} aria-label="Close chat">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
          
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div 
                key={message.id} 
                className={clsx(styles.message, {
                  [styles.userMessage]: message.role === 'user',
                  [styles.botMessage]: message.role === 'assistant'
                })}
              >
                {message.content}
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sourcesContainer}>
                    <p className={styles.sourcesTitle}>Sources:</p>
                    <ul className={styles.sourcesList}>
                      {message.sources.map((source, idx) => (
                        <li key={idx} className={styles.sourceItem}>
                          {source.title} (Page {source.page})
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {/* Removed separate current typing indicator since we stream directly into the last message */}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.inputArea} onSubmit={handleSendMessage}>
            <input
              type="text"
              className={styles.input}
              placeholder="Ask a question..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              disabled={isLoading}
            />
            <button 
              type="submit" 
              className={styles.sendButton} 
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </form>
        </div>
      )}

      <button className={styles.toggleButton} onClick={toggleChat} aria-label="Toggle chat">
        {isOpen ? (
           <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
             <line x1="18" y1="6" x2="6" y2="18"></line>
             <line x1="6" y1="6" x2="18" y2="18"></line>
           </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>
    </div>
  );
};

export default ChatWidget;
