# API Contract: RAG Chatbot Backend Service

**Feature**: RAG Chatbot Backend Service
**Date**: 2025-12-17
**Version**: 1.0.0

## Overview

This document specifies the API contracts for the RAG Chatbot Backend Service. The API follows REST principles and supports both standard responses and Server-Sent Events (SSE) for streaming.

## Base URL

```
https://ai-driven-textbook.vercel.app/api/rag
```

## Authentication

No authentication required for basic functionality. Session management is handled through client-generated session IDs.

## Endpoints

### 1. Health Check

#### GET /health

Check the health status of the RAG chatbot service.

**Request**:
```
GET /health
```

**Response**:
- `200 OK`: Service is healthy
  ```json
  {
    "status": "healthy",
    "timestamp": "2025-12-17T10:30:00Z"
  }
  ```

### 2. Standard Chat

#### POST /chat

Send a message to the chatbot and receive a response.

**Request**:
```
POST /chat
Content-Type: application/json
```

**Body**:
```json
{
  "message": "Can you explain URDF in simple terms?",
  "session_id": "session-abc123xyz",
  "context_text": "Optional highlighted text from the textbook"
}
```

**Response**:
- `200 OK`: Successful response
  ```json
  {
    "response": "URDF stands for Unified Robot Description Format...",
    "session_id": "session-abc123xyz",
    "sources": [
      {
        "title": "Robotics Chapter 3",
        "section": "URDF Overview",
        "page": 45
      }
    ]
  }
  ```
- `400 Bad Request`: Invalid request body
- `500 Internal Server Error`: Service unavailable

### 3. Streaming Chat

#### POST /chat/stream

Send a message to the chatbot and receive a streamed response using Server-Sent Events (SSE).

**Request**:
```
POST /chat/stream
Content-Type: application/json
```

**Body**:
```json
{
  "message": "Can you simplify this?",
  "session_id": "session-abc123xyz",
  "context_text": "Complex paragraph about inverse kinematics..."
}
```

**Response**:
- `200 OK`: With `Content-Type: text/event-stream`
  ```
  event: message
  data: {"token": "URDF"}

  event: message
  data: {"token": "stands"}

  event: message
  data: {"token": "for"}

  event: message
  data: {"token": "Unified"}

  event: message
  data: {"token": "Robot"}

  event: message
  data: {"token": "Description"}

  event: message
  data: {"token": "Format"}

  event: sources
  data: {"sources": [{"title": "Robotics Chapter 3", "section": "URDF Overview", "page": 45}]}

  event: end
  data: {"session_id": "session-abc123xyz"}

  ```

## Error Responses

All error responses follow this structure:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details if applicable"
  }
}
```

### Common Error Codes

- `INVALID_REQUEST`: Request body is malformed or missing required fields
- `SERVICE_UNAVAILABLE`: External service (Qdrant, Groq) is temporarily unavailable
- `SESSION_NOT_FOUND`: Provided session_id does not exist
- `RATE_LIMIT_EXCEEDED`: Request rate limit has been exceeded
- `INTERNAL_ERROR`: An internal server error occurred

## Data Models

### MessageRequest

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| message | string | Yes | The user's message to the chatbot |
| session_id | string | Yes | Unique identifier for the conversation session |
| context_text | string | No | Optional text context (e.g., highlighted text from textbook) |

### ChatResponse

| Field | Type | Description |
|-------|------|-------------|
| response | string | The chatbot's response to the user's message |
| session_id | string | The session identifier |
| sources | array[Source] | List of sources used to generate the response |

### Source

| Field | Type | Description |
|-------|------|-------------|
| title | string | Title of the source (e.g., textbook chapter) |
| section | string | Section name within the source |
| page | number | Page number in the source |

### StreamToken

| Field | Type | Description |
|-------|------|-------------|
| token | string | A token (word/sentence) in the response stream |