# Research: RAG Chatbot Backend Service

**Feature**: RAG Chatbot Backend Service
**Date**: 2025-12-17
**Input**: Feature spec from `/specs/003-rag-chatbot-backend/spec.md`, Implementation plan from `/specs/003-rag-chatbot-backend/plan.md`

## Overview

This research document addresses the unknowns and technical decisions for implementing the RAG Chatbot Backend Service as specified in the feature spec. The system will use FastAPI with async patterns to handle RAG queries to the Physical AI & Humanoid Robotics textbook content.

## Technical Decisions & Rationales

### 1. Python Version and Environment Management

**Decision**: Use Python 3.11 with uv for environment management
**Rationale**: Python 3.11 provides excellent performance improvements over previous versions. uv is a fast Python package installer and resolver, ensuring quick dependency installation and consistent environments.
**Alternatives considered**: Python 3.10, 3.12; pip + venv vs. uv vs. poetry
**Final choice**: Python 3.11 with uv for optimal development experience

### 2. FastAPI for Web Framework

**Decision**: Implement with FastAPI framework
**Rationale**: FastAPI's async-first design is ideal for I/O-heavy operations like LLM calls, vector DB queries, and PostgreSQL operations. It also provides automatic API documentation at `/docs` and `/redoc`.
**Alternatives considered**: Flask, Django, Starlette
**Final choice**: FastAPI for its async support and automatic documentation generation

### 3. Qdrant for Vector Database

**Decision**: Use Qdrant Cloud (Free Tier) for vector storage and retrieval
**Rationale**: Qdrant is specifically designed for vector similarity search, which is needed for the RAG component. The Cloud offering provides managed infrastructure and the free tier is sufficient for initial development and low-traffic usage.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB
**Final choice**: Qdrant for its performance and managed cloud offering

### 4. Neon Serverless PostgreSQL for Session Storage

**Decision**: Use Neon Serverless PostgreSQL for conversation memory
**Rationale**: Neon's serverless PostgreSQL provides automatic scaling, branching, and is PostgreSQL-compatible. This is ideal for storing conversation history while ensuring reliability and performance.
**Alternatives considered**: Standard PostgreSQL, Redis, SQLite
**Final choice**: Neon Serverless PostgreSQL for its serverless scaling and PostgreSQL compatibility

### 5. OpenAI Agents SDK for Agent Framework

**Decision**: Use OpenAI Agents SDK for the agent implementation
**Rationale**: The OpenAI Agents SDK provides a lightweight and powerful framework for creating agents that can work with various LLM providers, including Groq. It supports tool calling and memory management.
**Alternatives considered**: LangChain, CrewAI, custom agent implementation
**Final choice**: OpenAI Agents SDK for its lightweight nature and compatibility with Groq API

### 6. Groq API for LLM

**Decision**: Use Groq API with llama-3.1-8b-instant model
**Rationale**: Groq provides fast inference times which is crucial for a responsive chatbot experience. The free model offers good performance for development.
**Alternatives considered**: OpenAI API, Anthropic Claude, Hugging Face Inference API
**Final choice**: Groq API for speed and cost-effectiveness

### 7. Streaming Implementation

**Decision**: Use Server-Sent Events (SSE) for response streaming
**Rationale**: SSE provides a simple and efficient way to stream responses from the server to the frontend. It's well-supported in browsers and ideal for streaming responses from the LLM.
**Alternatives considered**: WebSockets, long polling, chunked transfer encoding
**Final choice**: SSE for its simplicity and browser support

### 8. Frontend Integration

**Decision**: Create a React-based floating chat widget for Docusaurus integration
**Rationale**: A React component can be easily integrated into the existing Docusaurus site. A floating widget provides a non-intrusive UI that can appear on any page of the textbook.
**Alternatives considered**: Vanilla JavaScript, Vue component, iframe integration
**Final choice**: React component for seamless integration with modern frontend tooling

### 9. Security and Configuration

**Decision**: Use Pydantic Settings for configuration management
**Rationale**: Pydantic Settings provides type validation and environment variable loading with minimal code. It's the standard approach for configuration in FastAPI applications.
**Alternatives considered**: Python's configparser, python-decouple, custom environment loading
**Final choice**: Pydantic Settings for its integration with FastAPI and type safety

### 10. Session Management

**Decision**: Client-side session ID generation with server-side storage
**Rationale**: Generates session IDs client-side using a crypto-safe random string and stores them in localStorage. This allows conversations to persist across page reloads without requiring authentication for basic use cases.
**Alternatives considered**: Server-side session management, JWT tokens, URL parameters
**Final choice**: Client-side generation with localStorage for simplicity and persistence

## Architecture Patterns

### RAG Implementation
- Uses embeddings to convert text queries to vectors
- Performs similarity search in Qdrant vector database
- Passes retrieved context to the LLM to generate responses
- Ensures all answers are grounded in textbook content

### Async Patterns
- All I/O operations (database queries, API calls, network requests) use async/await
- FastAPI endpoints are async functions to prevent blocking
- Efficient handling of concurrent users

### Error Handling
- Graceful degradation when external services (Qdrant, Groq) are unavailable
- User-friendly error messages
- Comprehensive logging for debugging

## Open Questions & Risks

### Resolved Questions
- Q: Which Python version to use? A: 3.11
- Q: How to handle async I/O operations? A: Use async/await with FastAPI
- Q: Which vector database to use? A: Qdrant Cloud
- Q: How to implement streaming responses? A: Server-Sent Events (SSE)
- Q: How to manage configuration? A: Pydantic Settings
- Q: How to store conversation history? A: Neon Serverless PostgreSQL

### Risks and Mitigations
- **Risk**: Rate limiting from Groq API
  - **Mitigation**: Implement proper error handling and retry logic
- **Risk**: Performance degradation with concurrent users
  - **Mitigation**: Use async patterns throughout and implement caching where appropriate
- **Risk**: Inaccurate responses from the LLM
  - **Mitigation**: Use RAG to ground responses in textbook content with proper citations