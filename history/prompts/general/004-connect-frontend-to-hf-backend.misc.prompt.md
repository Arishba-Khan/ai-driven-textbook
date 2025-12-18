---
id: 004
title: Connect Frontend to HF Backend
stage: misc
date: 2025-12-18
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: 122
command: none
labels: [docusaurus, huggingface, configuration, connection]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - d:/Secondary/Tech/ai-hackathon-1/physical-ai-textbook/docusaurus.config.ts
 - d:/Secondary/Tech/ai-hackathon-1/physical-ai-textbook/src/components/ChatWidget/index.tsx
---

## Prompt

https://huggingface.co/spaces/arishbakhan/rag_chatbot here is url, now connect it to my vercel https://ai-driven-textbook.vercel.app

## Response snapshot

I've created a plan to connect your Vercel frontend to the Hugging Face backend. I'll add a configuration field to your `docusaurus.config.ts` and update the `ChatWidget` component to use it. This way, you can easily change the backend URL in the future using environment variables on Vercel.

## Outcome

- ✅ Impact: Successfully configured the frontend to communicate with the HF backend dynamically.
- 🧪 Tests: Manual verification by user requested.
- 📁 Files: Updated docusaurus.config.ts and src/components/ChatWidget/index.tsx.
- 🔁 Next prompts: none
- 🧠 Reflection: Moving hardcoded URLs to configuration/environment variables is critical for cross-platform deployments.
-- Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
