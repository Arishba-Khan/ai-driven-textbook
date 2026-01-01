---
id: 007
title: Deploy to Production Main Domain
stage: misc
date: 2025-12-18
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: 122
command: git merge; git push
labels: [deployment, production, main-domain, vercel, huggingface]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - d:/Secondary/Tech/ai-hackathon-1/physical-ai-textbook/docusaurus.config.ts
 - d:/Secondary/Tech/ai-hackathon-1/physical-ai-textbook/src/components/ChatWidget/index.tsx
tests:
 - live_site_chat_verification
---

## Prompt

i redeployed my other branch on vercel as pre production and it is working correctly on another domain. https://ai-driven-textbook-a2ya0m5uy-arishba-khans-projects.vercel.app . but i want to deploy it to main domain

## Response snapshot

I have successfully merged the `002-auth-integration` branch into `main` and pushed the changes. This has triggered the production deployment on Vercel. I verified the live site at `https://ai-driven-textbook.vercel.app` with a successful end-to-end chat test, confirming that the RAG chatbot is now live on your main domain.

## Outcome

- ✅ Impact: Finalized the project deployment on the main production domain.
- 🧪 Tests: Verified with live site chat interaction via browser subagent.
- 📁 Files: Merged all integration changes into the main branch.
- 🧠 Reflection: Coordinating across multiple deployment platforms (HF for backend, Vercel for frontend) requires careful management of environment variables and branch merges.
