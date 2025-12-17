<!-- SYNC IMPACT REPORT:
     Version change: 1.1.0 → 2.0.0
     Removed sections: Original textbook-specific principles
     Added sections: RAG Chatbot Backend Service principles
     Templates requiring updates: ⚠ .specify/templates/plan-template.md, ⚠ .specify/templates/spec-template.md, ⚠ .specify/templates/tasks-template.md, ⚠ README.md
     Follow-up TODOs: Manual updates needed for all templates to reflect new backend focus
-->

# Physical AI & Humanoid Robotics RAG Chatbot Backend Constitution

## Core Principles

### Production-Ready Scalability
Build a production-ready, scalable backend that empowers the textbook with an intelligent tutor; The RAG chatbot must handle concurrent users with consistent response times and minimal downtime

### Beginner-Friendly Explanations
Generate clear, beginner-friendly explanations using Groq's fast LLMs; All responses must be accessible to students with Python and basic AI knowledge but no advanced robotics experience

### Intelligent Tutor Experience
Provide an intelligent tutoring system that enhances learning through interactive conversations; Students should feel supported by an AI tutor that understands their context and learning pace

### Accuracy and Groundedness
All answers must be firmly grounded in retrieved textbook content from the Qdrant vector database; The system must strictly cite sources and clearly state when it cannot find relevant information

## Architecture & Tech Stack Principles

### FastAPI Async-First Design
Use FastAPI for its async-first design, ideal for I/O-heavy operations like LLM calls and vector DB queries; All endpoints must leverage asynchronous patterns for optimal performance

### OpenAI Agents SDK Integration
Use the OpenAI Agents SDK for lightweight, powerful agent patterns compatible with Groq and other LLM providers; Implement robust agent workflows that handle various conversation scenarios

### Component Separation
Organize backend code in `/rag_backend` folder with clear separation of concerns between retrieval, generation, and session management; Each module must have well-defined interfaces and responsibilities

### Vector Database Excellence
Leverage Qdrant Cloud (Free Tier) for efficient vector storage and retrieval of textbook content; All embeddings must be pre-populated and properly indexed for optimal search performance

### Session Management
Utilize Neon Serverless PostgreSQL (Free Tier) for reliable conversation memory and session persistence; Ensure session data is stored securely with appropriate cleanup policies

## Frontend Integration Principles

### Clean API Design
Expose a clean API for a floating chat widget to be embedded in the Docusaurus site; API endpoints must follow RESTful principles with comprehensive documentation

### Real-time Communication
Implement streaming responses to the frontend for enhanced user experience; All API responses should include appropriate headers and status codes

### Dual-Mode RAG Support
Handle both general questions and questions about user-selected text from the book through a dual-mode RAG endpoint; The system must differentiate between these modes appropriately

## Mandatory Technical Standards

### Environment Configuration
All secrets (API keys, DB URLs) must be managed via environment variables using Pydantic settings; No hardcoded credentials or API keys are allowed in the source code

### Async Patterns Implementation
Utilize asynchronous patterns in FastAPI for non-blocking operations; All I/O-bound operations should be properly awaited to prevent blocking

### Performance Optimization
Optimize response times to ensure answers are delivered within seconds; Implement efficient caching mechanisms where appropriate to improve performance

### Error Handling
Implement comprehensive error handling with appropriate logging and graceful degradation; All error responses must be informative but not expose sensitive information

## Success Criteria

1. A student can highlight a complex paragraph about "URDF," ask "Can you simplify this?", and receive a clear, concise explanation based on the book's content within seconds
2. The backend service handles concurrent users without performance degradation
3. All responses are properly cited and grounded in textbook content
4. The API supports both general and context-specific queries seamlessly
5. Session memory persists across conversation turns appropriately
6. The system correctly indicates when it cannot find relevant information
7. Response streaming provides a smooth user experience
8. All security best practices are followed for credential management

## Governance

All PRs/reviews must verify constitution compliance for RAG backend implementation; Code must follow async-first principles with proper error handling; API endpoints must be documented and tested; Changes to database schema must be handled with proper migration strategies; Authentication integration must follow security best practices; All components must be deployable with appropriate monitoring and logging

**Version**: 2.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17