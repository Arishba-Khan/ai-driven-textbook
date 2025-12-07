# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-textbook-physical-ai/`
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

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan
- [X] T002 Initialize documentation framework with Docusaurus
- [X] T003 [P] Configure linting and formatting tools for Markdown content

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Setup documentation navigation structure per syllabus modules
- [X] T005 [P] Configure textbook styling with mobile and dark mode support
- [X] T006 [P] Setup basic chapter template following required structure
- [X] T007 Create base components for code examples and exercises
- [X] T008 Configure Mermaid diagram rendering
- [X] T009 Setup environment for content validation and build process

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Introduction Content (Priority: P1) 🎯 MVP

**Goal**: Create the complete Introduction section with 4 chapters containing syllabus content verbatim

**Independent Test**: Student can read the "Why Physical AI Matters", "Quarter Overview", "Learning Outcomes", and "Weekly Breakdown" sections and articulate the main goals and topics of the course.

### Implementation for User Story 1

- [X] T010 [P] [US1] Create `docs/intro/why-physical-ai.md` with FULL original paragraph verbatim from syllabus
- [X] T011 [P] [US1] Create `docs/intro/quarter-overview.md` with FULL original paragraph verbatim from syllabus
- [X] T012 [P] [US1] Create `docs/intro/learning-outcomes.md` with all 6 learning outcomes exactly as written
- [X] T013 [P] [US1] Create `docs/intro/weekly-breakdown.md` with Weeks 1-13 with EVERY sub-bullet exactly as written

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Learn ROS 2 Fundamentals (Priority: P1)

**Goal**: Create Module 1: The Robotic Nervous System (ROS 2) with 7 chapters covering all topics from nodes to URDF

**Independent Test**: Student can navigate Module 1 content, understand ROS 2 concepts like nodes, topics, and services, and run provided Python code examples successfully.

### Implementation for User Story 2

- [X] T014 [P] [US2] Create `docs/module-1/01-introduction-to-ros2.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T015 [P] [US2] Create `docs/module-1/02-ros2-nodes-deep-dive.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T016 [P] [US2] Create `docs/module-1/03-topics-and-publishers.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T017 [P] [US2] Create `docs/module-1/04-subscribers-and-callbacks.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T018 [P] [US2] Create `docs/module-1/05-services-request-response.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T019 [P] [US2] Create `docs/module-1/06-actions-long-running-tasks.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T020 [P] [US2] Create `docs/module-1/07-urdf-for-humanoid-robots.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 5 - Implement Vision-Language-Action Systems (Priority: P3)

**Goal**: Create Module 4: Vision-Language-Action (VLA) with 3 chapters, including the capstone project with full description verbatim

**Independent Test**: Student can navigate Module 4 content, understand VLA concepts, and implement autonomous humanoid systems successfully.

### Implementation for User Story 5

- [X] T021 [P] [US5] Create `docs/module-4/01-voice-to-action-openai-whisper.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T022 [P] [US5] Create `docs/module-4/02-llm-cognitive-planning-natural-language.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T023 [US5] Create `docs/module-4/03-capstone-autonomous-humanoid.md` with FULL capstone project description verbatim from syllabus and following required structure with 2-3 Python code examples, exercises, Mermaid diagram

**Checkpoint**: At this point, User Stories 1, 2 AND 5 should all work independently

---

## Phase 6: User Story 6 - Understand Hardware Requirements (Priority: P1)

**Goal**: Create Hardware Requirements chapter with all 6 specified tables exactly as provided in the syllabus

**Independent Test**: Student can review the hardware requirements section and understand the minimum specifications needed for different components of the system.

### Implementation for User Story 6

- [X] T024 [US6] Create `docs/hardware/index.md` containing all 6 hardware tables exactly as provided in syllabus:
- [X] T025 [P] [US6] Add Digital Twin Workstation Table (RTX 4070 Ti+, 64GB RAM, Ubuntu 22.04, etc.) verbatim
- [X] T026 [P] [US6] Add Physical AI Edge Kit Table (Jetson Orin Nano/NX, RealSense D435i, ReSpeaker, etc.) verbatim
- [X] T027 [P] [US6] Add Robot Lab Options A/B/C Table (Unitree Go2 Edu, G1, Hiwonder TonyPi Pro with prices) verbatim
- [X] T028 [P] [US6] Add Summary of Architecture Table (Sim Rig → Edge Brain → Sensors → Actuator) verbatim
- [X] T029 [P] [US6] Add Cloud-Native "Ether" Lab Table with full cost calculation (~$205/quarter) verbatim
- [X] T030 [P] [US6] Add Economy Jetson Student Kit Table (~$700 total with all components) verbatim

**Checkpoint**: At this point, User Stories 1, 2, 5 AND 6 should all work independently

---

## Phase 7: User Story 3 - Understand Digital Twin Simulations (Priority: P2)

**Goal**: Create Module 2: The Digital Twin (Gazebo & Unity) with 5 chapters covering simulation

**Independent Test**: Student can navigate Module 2 content, understand Gazebo/Unity concepts, and run simulation examples successfully.

### Implementation for User Story 3

- [X] T031 [P] [US3] Create `docs/module-2/01-introduction-to-gazebo.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T032 [P] [US3] Create `docs/module-2/02-urdf-sdf-robot-description.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T033 [P] [US3] Create `docs/module-2/03-physics-simulation-gravity-collisions.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T034 [P] [US3] Create `docs/module-2/04-sensor-simulation-lidar-depth-imu.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T035 [P] [US3] Create `docs/module-2/05-unity-high-fidelity-rendering.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram

**Checkpoint**: At this point, User Stories 1, 2, 3, 5 AND 6 should all work independently

---

## Phase 8: User Story 4 - Navigate with NVIDIA Isaac (Priority: P2)

**Goal**: Create Module 3: The AI-Robot Brain (NVIDIA Isaac™) with 4 chapters covering navigation and AI

**Independent Test**: Student can navigate Module 3 content, understand Isaac concepts, and implement navigation algorithms successfully.

### Implementation for User Story 4

- [X] T036 [P] [US4] Create `docs/module-3/01-introduction-to-nvidia-isaac-sim.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T037 [P] [US4] Create `docs/module-3/02-isaac-ros-vslam-navigation.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T038 [P] [US4] Create `docs/module-3/03-nav2-path-planning-bipedal.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram
- [X] T039 [P] [US4] Create `docs/module-3/04-sim-to-real-transfer-techniques.md` following required structure with 2-3 Python code examples, exercises, Mermaid diagram

**Checkpoint**: At this point, all user stories should be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T040 [P] Update sidebar navigation in `sidebars.ts` to include all created chapters
- [X] T041 Run `npm run build` to identify and fix any build errors
- [X] T042 Test mobile responsiveness across all created chapters
- [X] T043 Verify all 6 hardware tables match syllabus exactly
- [X] T044 Count total code examples to ensure minimum of 12 across all modules
- [X] T045 Verify all chapters follow the required structure (Overview → Core Concepts → Hands-on Examples → Key Takeaways → Quick Exercises)
- [X] T046 Ensure warm, encouraging tone throughout all content
- [X] T047 Verify all terminology is bolded and explained on first use
- [X] T048 Run final build test to ensure site builds without errors

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 6 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Each chapter follows the required structure: Overview → Core Concepts → Hands-on Examples → Key Takeaways → Quick Exercises
- All terminology is bolded and explained immediately on first use
- Each chapter includes 2-3 runnable Python code examples with expected output
- Each chapter includes 1-2 hands-on exercises with success criteria
- Each chapter includes at least 1 Mermaid diagram
- Each chapter maintains 800-1500 words (excluding code)
- Each chapter uses warm, encouraging, mentor-like tone

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All chapters within each user story can be created in parallel [P] since they're in different files
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 2 (ROS 2 Module)

```bash
# Launch all ROS 2 chapters in parallel:
Task: "Create docs/module-1/01-introduction-to-ros2.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/02-ros2-nodes-deep-dive.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/03-topics-and-publishers.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/04-subscribers-and-callbacks.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/05-services-request-response.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/06-actions-long-running-tasks.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
Task: "Create docs/module-1/07-urdf-for-humanoid-robots.md following required structure with 2-3 Python code examples, exercises, Mermaid diagram"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 5, 6 - the Critical Path)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Intro content)
4. Complete Phase 4: User Story 2 (ROS 2 - foundational)
5. Complete Phase 5: User Story 5 (VLA - includes capstone)
6. Complete Phase 6: User Story 6 (Hardware - critical requirements)
7. **STOP and VALIDATE**: Ensure all critical content is present and meets constitution requirements independently
8. Publish/demo if ready

### Incremental Delivery

1. Complete MVP (US1, US2, US5, US6) → Core textbook ready
2. Add User Story 3 (Gazebo/Unity module) → Extended content
3. Add User Story 4 (NVIDIA Isaac module) → Complete textbook
4. Complete Phase N: Polish → Production ready
5. Each addition maintains textbook quality standards

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Creator A: User Story 1 (Intro: 4 chapters)
   - Creator B: User Story 2 (Module 1: 7 ROS 2 chapters)
   - Creator C: User Story 5 (Module 4: 3 VLA chapters + capstone)
   - Creator D: User Story 6 (Hardware: 1 chapter with 6 tables)
   - Creator E: User Story 3 (Module 2: 5 Gazebo/Unity chapters)
   - Creator F: User Story 4 (Module 3: 4 NVIDIA Isaac chapters)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Each chapter must follow the exact structure per requirements
- All 6 hardware tables must match syllabus exactly
- Minimum 12 code examples must be distributed across all modules
- All content must maintain warm, encouraging mentor tone
- Verify site builds without errors before finalizing