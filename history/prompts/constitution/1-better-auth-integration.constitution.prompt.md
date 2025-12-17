---
id: 1
title: better-auth-integration
stage: constitution
date: 2025-12-15
surface: agent
model: Qwen
feature: auth-integration
branch: main
user: user
command: /sp.constitution Create a constitution for integrating Better Auth authentication
labels: ["authentication", "monorepo", "deployment", "security"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - history/prompts/constitution/1-better-auth-integration.constitution.prompt.md
tests:
 - none
---

## Prompt

/sp.constitution Create a constitution for integrating Better Auth authentication into the Physical AI & Humanoid Robotics Docusaurus textbook using a SINGLE VERCEL DEPLOYMENT.

**Core Principle:** Everything (Docusaurus frontend + Auth backend + Vercel config) must be created in ONE repository and deployable with ONE Git push to Vercel.

**Architecture Rules:**
1. **Monorepo Structure:** Auth backend lives in `/auth-backend` folder within the main Docusaurus project
2. **Single Deployment:** Both frontend and backend deploy together to Vercel via root `vercel.json`
3. **No CORS:** Frontend (ai-driven-textbook.vercel.app) and backend (ai-driven-textbook.vercel.app/api/auth) share same domain
4. **Serverless Functions:** Auth backend runs as Vercel Serverless Functions, not separate server

**Mandatory Deliverables (AI Must Create All):**
- ✅ `/auth-backend/` folder with full Node.js/TypeScript Better Auth server
- ✅ Root `/vercel.json` config for monorepo deployment
- ✅ `/auth-backend/vercel.json` config for serverless functions
- ✅ Updated root `package.json` with build scripts for both projects
- ✅ Frontend auth components in Docusaurus `src/`
- ✅ `/signup` and `/signin` pages with hardware/software background questions

**Success Metric:** After `git push`, Vercel automatically builds and deploys complete site with working auth at `https://ai-driven-textbook.vercel.app`

**Hackathon Requirement:** Translation button only visible to logged-in users who signed up with background info.

## Response snapshot

Updated the constitution file to incorporate the Better Auth authentication requirements while preserving the existing principles. Added new principles related to authentication and deployment architecture.

## Outcome

- ✅ Impact: Added auth-specific principles to constitution while preserving existing educational-focused principles
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md and created PHR
- 🔁 Next prompts: Create auth backend implementation based on new constitution
- 🧠 Reflection: Successfully integrated both the educational content requirements and the technical authentication requirements into a unified constitution

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation of auth backend in /auth-backend folder
- Manual verification: Checked that all requirements from the original request were incorporated into the new constitution