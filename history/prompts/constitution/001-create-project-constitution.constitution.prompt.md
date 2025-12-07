---
id: 001
title: Create Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, textbook, setup]
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
  - constitution validation
---

## Prompt

```
Create a constitution for a provided syllabus.

## **Core Writing Principles:**
1. **Clarity First:** Crystal-clear explanations using simple English for complete beginners in robotics
2. **Encouraging Tone:** Warm, mentor-like, supportive throughout - like a helpful teacher who believes in the student
3. **Logical Progression:** Foundations (ROS 2) → Tools (Gazebo/Isaac) → Real Applications (VLA Capstone)
4. **Hands-on Mandate:** EVERY major topic MUST include complete, copy-paste-ready code examples
5. **Zero Fluff, Maximum Value:** Every word must teach - no filler content

## **Content Standards (NON-NEGOTIABLE):**
- **Chapter Structure:** Overview → Core Concepts → Hands-on Examples → Key Takeaways → Quick Exercises
- **Terminology:** First use is **bolded** and explained immediately in simple terms
- **Visuals:** All diagrams in Mermaid, all tables in clean Markdown
- **Code Examples:** Always testable, with clear comments and expected output shown
- **Hardware Tables:** MUST be reproduced VERBATIM from the syllabus - no changes allowed
- **Tone:** Supportive mentor voice throughout - "You've got this — let's build it together!"

## **Required Content Coverage (100% MANDATORY):**
- **Introduction Sections:** Why Physical AI Matters, Quarter Overview, Learning Outcomes, Weekly Breakdown - ALL VERBATIM
- **Module 1:** The Robotic Nervous System (ROS 2) - 7 chapters
- **Module 2:** The Digital Twin (Gazebo & Unity) - 5 chapters  
- **Module 3:** The AI-Robot Brain (NVIDIA Isaac™) - 4 chapters
- **Module 4:** Vision-Language-Action (VLA) - 3 chapters
- **Hardware Chapter:** ALL 6 tables from syllabus exactly as i will provide

## **Target Audience:**
Students who already know Python and basic AI agents, but are COMPLETE BEGINNERS in robotics. Must work for both overwhelmed beginners and those wanting deeper details.

## **Success Criteria (ALL MUST BE TRUE):**
- [ ] 100% syllabus coverage - every bullet from syllabus appears as its own section
- [ ] Every chapter follows the exact required structure
- [ ] All hardware tables reproduced CHARACTER-FOR-CHARACTER
- [ ] Minimum 12 complete, copy-paste-ready code examples
- [ ] Site builds perfectly with zero errors
- [ ] Mobile experience is flawless, dark mode perfect
- [ ] Warm, encouraging tone maintained from first page to last
- [ ] A student with Python + basic AI knowledge can confidently start building humanoid robots after reading
```

## Response snapshot

A complete constitution has been created for the Physical AI & Humanoid Robotics textbook project with the following principles:

1. Clarity First - Crystal-clear explanations for beginners
2. Encouraging Tone - Supportive mentor voice throughout
3. Hands-on Mandate - Copy-paste-ready code examples
4. Zero Fluff, Maximum Value - Every word must teach
5. Verbatim Content Requirements - Hardware tables reproduced exactly
6. Complete Syllabus Coverage - 100% coverage mandatory

The constitution has been implemented in `.specify/memory/constitution.md` with version 1.0.0, and dependent templates have been updated accordingly.

## Outcome

- ✅ Impact: Established core principles for the Physical AI textbook project
- 🧪 Tests: Manual validation of constitution consistency
- 📁 Files: Updated constitution file and 4 dependent templates
- 🔁 Next prompts: /sp.plan, /sp.spec to begin development
- 🧠 Reflection: The constitution provides a strong foundation for consistent content creation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin content creation following constitution guidelines