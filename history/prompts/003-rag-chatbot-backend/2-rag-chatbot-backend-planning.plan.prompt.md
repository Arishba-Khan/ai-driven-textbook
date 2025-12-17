---
id: 2
title: RAG Chatbot Backend Planning
stage: plan
date: 2025-12-17
surface: agent
model: 
feature: 003-rag-chatbot-backend
branch: 
user: 
command: /sp.plan Create an execution plan for the RAG Chatbot based on the constitution.

**Implementation Phases:**

**Phase 1: Core Backend Service**
1.  **Foundation:** Create the `rag_backend/` directory structure with `pyproject.toml` and environment template. Refactor the user's agent script into the core modules: `retriever.py`, `agent_builder.py`, and `memory.py`.
2.  **FastAPI Layer:** Build the main FastAPI application (`main.py`) with CORS configured. Create the API router (`chat.py`) with initial endpoints.
3.  **Memory Integration:** Implement the `NeonPostgresMemory` class and integrate it with the agent for session-based conversation history.

**Phase 2: Frontend Integration & Streaming**
1.  **Streaming Endpoint:** Implement the `POST /chat/stream` endpoint in `chat.py` using Server-Sent Events (SSE) to stream the agent's response token-by-token.
2.  **Floating Chat Widget:** Create the React component (`ChatWidget.jsx`) for the Docusaurus frontend. It must be a toggleable floating button/panel that communicates with the streaming endpoint and displays the conversation.

**Phase 3: Deployment & Final Integration**
1.  **Vercel Configuration:** Update the project's root `vercel.json` to route requests (e.g., `/api/rag/*`) to the new Python backend. Ensure `rag_backend/vercel.json` is correctly configured.
2.  **Environment Setup:** Document and set all required environment variables (Groq, Qdrant, Cohere, Neon) in the Vercel project dashboard.
3.  **End-to-End Validation:** Perform a full test on the live deployment, verifying the widget appears, streams answers, remembers conversation context, and correctly uses selected text.

**Dependencies:**
*   The frontend widget (Phase 2.2) requires the streaming endpoint (Phase 2.1) to be complete.
*   Deployment (Phase 3) requires all backend and frontend code from Phases 1 and 2 to be complete.

**Risk Mitigation:**
*   **Agent Logic Errors:** Build and test the refactored retriever and agent modules in isolation first.
*   **Streaming Complexity:** Implement the non-streaming `POST /chat` endpoint first as a simpler milestone before adding SSE.
labels: [plan,rag,chatbot,backend,fastapi]
links:
  spec: specs/003-rag-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-rag-chatbot-backend/plan.md
 - specs/003-rag-chatbot-backend/research.md
 - specs/003-rag-chatbot-backend/data-model.md
 - specs/003-rag-chatbot-backend/quickstart.md
 - specs/003-rag-chatbot-backend/contracts/api-contract.md
 - QWEN.md
tests:
 - 
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType qwen`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


## Response snapshot

Successfully created the implementation plan for the RAG Chatbot Backend Service. The plan includes:

1. Implementation Plan document (plan.md) with technical context
2. Research document (research.md) with technical decisions and rationales
3. Data model document (data-model.md) with entity definitions
4. API contract document (contracts/api-contract.md) specifying endpoints
5. Quickstart guide (quickstart.md) for developers
6. Updated agent context file (QWEN.md)

The plan follows the constitution requirements, focusing on production-ready scalability, beginner-friendly explanations, async-first design, and clean API principles. All artifacts are located in the specs/003-rag-chatbot-backend directory.

## Outcome

- ✅ Impact: Comprehensive planning artifacts created for RAG Chatbot implementation
- 🧪 Tests: N/A for planning phase
- 📁 Files: Multiple planning documents created (plan.md, research.md, data-model.md, quickstart.md, contracts/api-contract.md, and updated QWEN.md)
- 🔁 Next prompts: Ready for task generation (`/sp.tasks`) to implement the design
- 🧠 Reflection: The planning phase has established a solid foundation with clear architecture, data models, API contracts, and implementation approach

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks with `/sp.tasks` to begin implementation