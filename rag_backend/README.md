# RAG Chatbot Backend Service

RAG Chatbot Backend Service for Physical AI & Humanoid Robotics Textbook

## Overview

This service provides an AI-powered chatbot that can answer questions about the Physical AI & Humanoid Robotics textbook using Retrieval-Augmented Generation (RAG). The system retrieves relevant content from the textbook and generates contextual responses using a language model.

## Features

- **RAG-based Q&A**: Answers questions based on textbook content
- **Session Management**: Maintains conversation history across multiple interactions
- **Streaming Responses**: Real-time response streaming using Server-Sent Events
- **Error Handling**: Graceful degradation when services are unavailable
- **Secure Configuration**: Environment variable management for API keys

## Architecture

- **Framework**: FastAPI
- **Language Model**: Groq API with llama3 model
- **Vector Database**: Qdrant for textbook content storage and retrieval
- **Session Storage**: Neon Serverless PostgreSQL
- **Embeddings**: Cohere for generating text embeddings

## Endpoints

- `GET /health`: Health check endpoint
- `POST /chat/`: Standard chat endpoint
- `POST /chat/stream`: Streaming chat endpoint with Server-Sent Events

## Setup and Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -e .
   ```
4. Create a `.env` file with your API keys based on `.env.example`:
   ```bash
   cp .env.example .env
   # Edit .env with your actual API keys
   ```
5. Start the development server:
   ```bash
   uvicorn app.main:app --reload --port 8000
   ```

## Environment Variables

- `GROQ_API_KEY`: API key for Groq LLM service
- `COHERE_API_KEY`: API key for Cohere embeddings service
- `QDRANT_API_KEY`: API key for Qdrant vector database
- `QDRANT_URL`: URL for Qdrant instance
- `QDRANT_COLLECTION_NAME`: Name of the collection storing textbook content (default: "humanoid_ai_book")
- `NEON_DB_URL`: Connection string for Neon PostgreSQL database
- `DEBUG_MODE`: Enable debug mode (default: false)
- `LOG_LEVEL`: Logging level (default: "INFO")

## API Usage

### Standard Chat

```bash
curl -X POST http://localhost:8000/chat/ \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you explain URDF in simple terms?",
    "session_id": "session-abc123xyz"
  }'
```

### Streaming Chat

```bash
curl -X POST http://localhost:8000/chat/stream \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you explain URDF in simple terms?",
    "session_id": "session-abc123xyz"
  }'
```

## Project Structure

```
rag_backend/
├── __init__.py
├── pyproject.toml          # Project dependencies and configuration
├── .env.example           # Example environment variables file
├── app/
│   ├── __init__.py
│   ├── main.py            # FastAPI application and server setup
│   ├── core/              # Core business logic
│   │   ├── __init__.py
│   │   ├── config.py      # Configuration and settings management
│   │   ├── agent_builder.py # AI agent implementation
│   │   ├── retriever.py   # Qdrant integration for RAG
│   │   └── memory.py      # Session and conversation memory
│   ├── api/               # API endpoints
│   │   ├── __init__.py
│   │   └── chat.py        # Chat API router and endpoints
│   └── models/            # Pydantic data models
│       ├── __init__.py
│       ├── message.py     # Message data model
│       ├── session.py     # Session data model
│       ├── retrieval.py   # Retrieval data model
│       └── error.py       # Error response models
├── api/                   # Vercel deployment entry point
│   ├── __init__.py
│   └── index.py           # Mangum adapter for serverless deployment
├── tests/                 # Test directory (structure)
│   ├── __init__.py
│   ├── unit/
│   └── integration/
└── vercel.json            # Vercel deployment configuration
```

## Deployment

The service is configured for deployment on Vercel. The configuration includes:

- Python 3.11 runtime
- API route handling via Mangum adapter
- Environment variable configuration for secrets

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

MIT License