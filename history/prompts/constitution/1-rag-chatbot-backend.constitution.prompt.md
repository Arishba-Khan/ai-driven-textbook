---
id: 1
title: RAG Chatbot Backend Constitution Update
stage: constitution
date: 2025-12-17
surface: agent
model: 
feature: rag-backend
branch: 
user: 
command: /sp.constitution Create a constitution for building a Retrieval-Augmented Generation (RAG) chatbot as a backend service for the Physical AI & Humanoid Robotics Docusaurus textbook
labels: [constitution,rag,backend,ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
 - README.md
tests:
 - 
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

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


/sp.constitution Create a constitution for building a Retrieval-Augmented Generation (RAG) chatbot as a backend service for the Physical AI & Humanoid Robotics Docusaurus textbook.

**Core Principle:** Build a production-ready, scalable backend that empowers the textbook with an intelligent tutor. The chatbot must answer questions by retrieving context from the pre-populated Qdrant vector database and generating clear, beginner-friendly explanations using Groq's fast LLMs.

**Architecture & Tech Stack Rules:**
- **Location:** All backend code resides in a `/rag_backend` folder at the project root in uv enviorment.
- **Framework:** Use FastAPI for its async-first design, ideal for I/O-heavy operations like LLM calls and vector DB queries[citation:1].
- **Agent Framework:** Use the OpenAI Agents SDK for its lightweight, powerful agent patterns and its compatibility with Groq and other LLM providers[citation:3].
- **Core Components:**
  - **Vector DB:** Qdrant Cloud (Free Tier), already populated with book embeddings.
  - **LLM:** Groq API (using a free model like `llama-3.1-8b-instant` for speed).
  - **Session Store:** Neon Serverless PostgreSQL (Free Tier) for conversation memory.
- **Frontend Integration:** The backend exposes a clean API for a floating chat widget to be embedded in the Docusaurus site.

**Mandatory Features:**
1. **Dual-Mode RAG Endpoint:** An API that handles both general questions and questions about user-selected text from the book.
2. **Conversation Memory:** Maintains session history using Neon Postgres, allowing for coherent multi-turn dialogues[citation:4].
3. **Accuracy & Safety:** Answers must be grounded in the retrieved textbook content. The system must cite sources and clearly state when it cannot find an answer.
4. **Performance:** Utilize asynchronous patterns in FastAPI for non-blocking operations[citation:1]. Responses should be streamed to the frontend for a better user experience.
5. **Robust Configuration:** All secrets (API keys, DB URLs) must be managed via environment variables using Pydantic settings.

**Success Metric:** A student can highlight a complex paragraph about "URDF," ask "Can you simplify this?", and receive a clear, concise explanation based on the book's content within seconds.

## Response snapshot

The constitution has been successfully updated for the RAG chatbot backend service. Here's the summary:

**New Version**: 2.0.0 (Major version bump due to fundamental shift from textbook-focused to RAG backend service-focused constitution)

**Version Bump Rationale**: This is a major change as the entire focus of the project has shifted from a static textbook to a dynamic RAG chatbot backend service. The core principles, architecture decisions, and success criteria have been completely redefined to match the new requirements.

**Files Updated**:
1. `.specify/memory/constitution.md` - Updated with new RAG backend principles and governance
2. `.specify/templates/plan-template.md` - Updated constitution check to reflect new backend principles
3. `.specify/templates/spec-template.md` - Updated functional requirements and success criteria for backend service
4. `.specify/templates/tasks-template.md` - Updated sample tasks to reflect backend implementation
5. `README.md` - Updated constitution principles summary

**Files Flagged for Manual Follow-up**:
- `.specify/templates/adr-template.md` - May need updates for backend architecture decisions
- `.specify/templates/agent-file-template.md` - May need updates for agent-specific implementation guidance
- Any other template files that might reference textbook-specific principles

**Suggested Commit Message**: `docs: amend constitution to v2.0.0 (RAG backend service implementation)`

The updated constitution now properly reflects the requirements for building a Retrieval-Augmented Generation (RAG) chatbot backend service, with emphasis on production-ready scalability, beginner-friendly explanations, intelligent tutor experience, accuracy and groundedness, FastAPI async-first design, and clean API design.

## Outcome

- ✅ Impact: Successfully updated the project constitution from textbook-focused to RAG backend service-focused, with associated template updates
- 🧪 Tests: No specific tests required for constitution update
- 📁 Files: 5 files updated to maintain consistency with new constitution
- 🔁 Next prompts: Implementation of RAG backend components can now proceed according to the new constitution
- 🧠 Reflection: The constitution now properly reflects the project's shift to a dynamic RAG backend service, providing clear guidance for implementation

## Evaluation notes (flywheel)

- Failure modes observed: None observed
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the RAG backend components according to the new constitution