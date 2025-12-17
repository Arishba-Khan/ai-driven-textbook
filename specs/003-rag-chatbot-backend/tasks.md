---

description: "Task list for RAG Chatbot Backend Service implementation"
---

# Tasks: RAG Chatbot Backend Service

**Input**: Design documents from `/specs/003-rag-chatbot-backend/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create rag_backend/ directory structure with pyproject.toml and .env.example
- [X] T002 [P] Install and configure dependencies: FastAPI, OpenAI Agents SDK, Qdrant, Pydantic, Groq API client
- [X] T003 [P] Set up basic project configuration using Pydantic Settings

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T004 Setup configuration management with Pydantic Settings for API keys
- [X] T005 [P] Configure Qdrant connection and test client integration
- [X] T006 [P] Configure Neon Postgres connection and test database client
- [X] T007 Create base data models for Session and Message entities
- [X] T008 [P] Implement basic QdrantRetriever class with embedding functionality
- [X] T009 [P] Implement basic NeonPostgresMemory class with session storage
- [X] T010 Setup CORS middleware for Docusaurus frontend integration

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Asks Questions About Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about textbook content and receive AI-generated responses grounded in the textbook

**Independent Test**: Student can highlight text, ask for clarification, and receive an accurate explanation within 2 seconds

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T011 [P] [US1] Contract test for POST /chat endpoint in tests/contract/test_chat.py
- [ ] T012 [P] [US1] Integration test for Qdrant retrieval in tests/integration/test_retrieval.py

### Implementation for User Story 1

- [X] T013 [P] [US1] Create Message model in rag_backend/models/message.py
- [X] T014 [P] [US1] Create Session model in rag_backend/models/session.py
- [X] T015 [US1] Implement complete QdrantRetriever class in rag_backend/core/retriever.py
- [X] T016 [US1] Implement complete NeonPostgresMemory class in rag_backend/core/memory.py
- [X] T017 [US1] Create AgentBuilder in rag_backend/core/agent_builder.py
- [X] T018 [US1] Create basic chat endpoint in rag_backend/api/chat.py
- [X] T019 [US1] Add session-based conversation history to agent responses
- [X] T020 [US1] Add error handling and response formatting

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Student Engages in Multi-Turn Dialogue  (Priority: P2)

**Goal**: Maintain conversation context so students can have multi-turn dialogues with the chatbot

**Independent Test**: Student can have a multi-turn conversation where the system remembers previous exchanges and provides contextually appropriate responses

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T021 [P] [US2] Contract test for session persistence in tests/contract/test_session.py
- [ ] T022 [P] [US2] Integration test for conversation history in tests/integration/test_conversation.py

### Implementation for User Story 2

- [X] T023 [P] [US2] Enhance Session model with last_interaction_at and active status
- [X] T024 [US2] Implement conversation history management in NeonPostgresMemory
- [X] T025 [US2] Implement session expiration logic in rag_backend/core/memory.py
- [X] T026 [US2] Update agent to utilize full conversation history context
- [X] T027 [US2] Add session context to message responses
- [X] T028 [US2] Implement session cleanup functionality for expired sessions

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - System Handles Various Error Scenarios (Priority: P3)

**Goal**: Provide graceful error handling when external services are unavailable

**Independent Test**: When external services are unavailable, the system provides helpful error messages to users rather than failing silently

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T029 [P] [US3] Contract test for error responses in tests/contract/test_error_handling.py
- [ ] T030 [P] [US3] Integration test for service failure scenarios in tests/integration/test_error_scenarios.py

### Implementation for User Story 3

- [X] T031 [P] [US3] Create error response models in rag_backend/models/error.py
- [X] T032 [US3] Implement error handling middleware in rag_backend/main.py
- [X] T033 [US3] Add retry logic for Qdrant and Groq API calls
- [X] T034 [US3] Implement graceful degradation responses when services unavailable
- [X] T035 [US3] Add comprehensive logging for error scenarios
- [X] T036 [US3] Update endpoints to return user-friendly error messages

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase 6: Frontend Integration

**Goal**: Create a React-based floating chat widget for the Docusaurus frontend that communicates with the backend

- [ ] T037 Create ChatWidget React component in frontend/src/components/ChatWidget.jsx
- [ ] T038 [P] Implement session ID generation and localStorage management
- [X] T039 Implement POST /chat/stream endpoint using Server-Sent Events in rag_backend/api/chat.py
- [ ] T040 Connect ChatWidget to streaming endpoint and handle response streaming
- [ ] T041 Add support for passing selected/highlighted text to the backend
- [ ] T042 Style and position the floating chat widget appropriately

---

## Phase 7: Deployment & Integration

**Goal**: Deploy the RAG backend and integrate with the existing Docusaurus site

- [X] T043 Update root vercel.json to route /api/rag/* to the new Python backend
- [X] T044 Create rag_backend/vercel.json configuration for Vercel deployment
- [ ] T045 Set up environment variables in Vercel dashboard for all API keys
- [X] T046 Implement health check endpoint at /api/rag/health
- [ ] T047 Add the ChatWidget to the Docusaurus site layout
- [ ] T048 Perform end-to-end testing on deployed site

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T049 [P] Documentation updates in docs/
- [ ] T050 Code cleanup and refactoring
- [ ] T051 Performance optimization across all stories
- [ ] T052 [P] Additional unit tests (if requested) in tests/unit/
- [ ] T053 Security hardening
- [X] T054 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Frontend Integration (Phase 6)**: Depends on User Story 1 completion
- **Deployment (Phase 7)**: Depends on all previous phases completion
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before API endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for POST /chat endpoint in tests/contract/test_chat.py"
Task: "Integration test for Qdrant retrieval in tests/integration/test_retrieval.py"

# Launch all models for User Story 1 together:
Task: "Create Message model in rag_backend/models/message.py"
Task: "Create Session model in rag_backend/models/session.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Ensure User Story 1 meets all constitution requirements independently
5. Deploy and test basic functionality

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Validate constitution compliance → Deploy basic chat (MVP!)
3. Add User Story 2 → Validate constitution compliance → Deploy multi-turn support
4. Add User Story 3 → Validate constitution compliance → Deploy error handling
5. Add frontend integration → Validate user experience → Deploy widget
6. Add deployment integration → Validate end-to-end → Full deployment
7. Each story adds value while maintaining RAG service quality standards

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core chat functionality)
   - Developer B: User Story 2 (session management)
   - Developer C: User Story 3 (error handling)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence