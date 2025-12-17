# Data Model: RAG Chatbot Backend Service

**Feature**: RAG Chatbot Backend Service
**Date**: 2025-12-17
**Input**: Feature spec from `/specs/003-rag-chatbot-backend/spec.md`, Research from `/specs/003-rag-chatbot-backend/research.md`

## Overview

This document defines the data models for the RAG Chatbot Backend Service. The models represent the entities and their relationships that will be used in the system, focusing on conversation management and message storage.

## Entity Models

### Session

**Description**: Represents a conversation thread between a user and the chatbot.

**Fields**:
- `id` (str): Unique identifier for the session, generated client-side and stored in localStorage
- `created_at` (datetime): Timestamp when the session was created
- `last_interaction_at` (datetime): Timestamp of the last interaction in the session
- `active` (bool): Whether the session is currently active (default: true)

**Relationships**:
- One-to-many with Message (one session contains many messages)

**Validation**:
- `id` must be a valid UUID or similar unique string
- `created_at` and `last_interaction_at` must be valid datetime objects
- Session should expire after inactivity period (e.g., 24 hours)

### Message

**Description**: Represents a single exchange in a conversation, either from the user or the assistant.

**Fields**:
- `id` (int/uuid): Unique identifier for the message
- `session_id` (str): Foreign key linking to the Session
- `role` (str): The role of the sender - either "user" or "assistant"
- `content` (str): The text content of the message
- `timestamp` (datetime): When the message was created
- `metadata` (dict): Additional information like source citations, retrieval context, etc.

**Relationships**:
- Many-to-one with Session (many messages belong to one session)

**Validation**:
- `role` must be one of "user" or "assistant"
- `content` must not be empty
- `timestamp` must be a valid datetime object
- `session_id` must reference an existing session

### Retrieval

**Description**: Represents a search result from the textbook that was used to inform an assistant response.

**Fields**:
- `id` (int/uuid): Unique identifier for the retrieval
- `query` (str): The original query that was embedded and searched
- `text_content` (str): The text content retrieved from the vector database
- `source_metadata` (dict): Information about where the content came from in the textbook (e.g., chapter, section, page)
- `relevance_score` (float): A score indicating how relevant the retrieved content is to the query
- `created_at` (datetime): When this retrieval was made

**Relationships**:
- Many-to-many with Message (an assistant message may be based on multiple retrievals)
- Stored separately to allow for analysis of retrieval quality

**Validation**:
- `query` and `text_content` must not be empty
- `relevance_score` must be between 0 and 1
- `source_metadata` must contain required citation information

## State Transitions

### Session States
- **Active**: Session has been created and is receiving messages
- **Inactive**: Session has not received messages for a configured period (e.g., 24 hours)
- **Archived**: Session has been explicitly archived by the system after a longer period

### Session Transitions
- Active → Inactive: After a period of inactivity (e.g., 24 hours)
- Inactive → Active: When a new message is added to the session
- Inactive → Archived: After a longer period of inactivity (e.g., 30 days)

## Relationships

```
Session (1) ─── (Many) Message
Session (1) ─── (Many) Retrieval (through assistant messages)
```

## Constraints & Business Rules

1. **Session Uniqueness**: Each session ID should be unique across the system
2. **Message Ordering**: Messages in a session should maintain chronological order based on timestamp
3. **Content Validation**: All message content should be validated for appropriateness before storage
4. **Retrieval Linking**: Each assistant message should be linked to the retrievals used to generate it
5. **Session Expiration**: Sessions should be automatically marked as inactive after inactivity
6. **Data Retention**: Older sessions may be archived to optimize database performance

## Indexes

- Session: Index on `id`, `last_interaction_at`
- Message: Index on `session_id`, `timestamp`
- Retrieval: Index on `query` (for similarity matching)