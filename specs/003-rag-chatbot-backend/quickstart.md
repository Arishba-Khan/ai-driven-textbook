# Quickstart Guide: RAG Chatbot Backend Service

**Feature**: RAG Chatbot Backend Service
**Date**: 2025-12-17

## Overview

This guide provides a quick introduction to setting up and running the RAG Chatbot Backend Service. The service enables intelligent tutoring capabilities by retrieving relevant information from the Physical AI & Humanoid Robotics textbook and generating contextual responses.

## Prerequisites

- Python 3.11+
- uv (for environment management)
- Access to the following APIs:
  - Groq API (for LLM)
  - Qdrant Cloud (for vector database)
  - Neon Serverless PostgreSQL (for session storage)

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Python Environment

```bash
# Install uv if you don't have it
pip install uv

# Create and activate virtual environment
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies

```bash
# From the rag_backend directory
cd rag_backend
uv pip install -r requirements.txt
```

### 4. Configure Environment Variables

Copy the example environment file and add your API keys:

```bash
cp .env.example .env
```

Then edit `.env` to add your API keys:

```bash
GROQ_API_KEY="your-groq-api-key"
QDRANT_API_KEY="your-qdrant-api-key"
QDRANT_URL="your-qdrant-url"
NEON_DB_URL="your-neon-db-url"
COHERE_API_KEY="your-cohere-api-key"
```

### 5. Start the Development Server

```bash
cd app
python -m main
```

The API will be available at `http://localhost:8000`.

## Basic Usage

### Send a Message

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you explain URDF in simple terms?",
    "session_id": "session-abc123xyz"
  }'
```

### Stream a Response

```bash
curl -X POST http://localhost:8000/chat/stream \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you explain URDF in simple terms?",
    "session_id": "session-abc123xyz"
  }'
```

## API Endpoints

- `GET /health`: Check service health
- `POST /chat`: Standard chat endpoint
- `POST /chat/stream`: Streaming chat endpoint

## Architecture Overview

The RAG Chatbot service consists of:

1. **Core Components**:
   - `retriever.py`: Handles embedding queries and searching the Qdrant vector database
   - `agent_builder.py`: Creates and configures the AI agent with Groq LLM
   - `memory.py`: Implements session management with Neon PostgreSQL

2. **API Layer**:
   - `main.py`: FastAPI application with CORS setup
   - `chat.py`: API router with chat endpoints

3. **Data Flow**:
   - User message → Text embedding → Qdrant search → Context retrieval → LLM response → Session storage

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify all environment variables are correctly set
2. **Database Connection Error**: Check Neon DB URL and credentials
3. **Qdrant Connection Error**: Verify Qdrant URL and API key

### Getting Help

- Check the full API documentation at `/docs`
- Review logs for specific error messages
- Ensure all prerequisites are properly installed and configured