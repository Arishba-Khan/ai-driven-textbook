---
id: 2
title: auth-integration-spec
stage: spec
date: 2025-12-15
surface: agent
model: Qwen
feature: auth-integration
branch: 002-auth-integration
user: user
command: /sp.specify Create detailed specification based on the monorepo constitution. **Project: All-in-One Auth Integration for Vercel** **1. Backend Server Requirements (`/auth-backend/server.ts`):** - Use Express + Better Auth with `@better-auth/express` - Custom user fields: `experienceLevel`, `programmingLanguages`, `roboticsExperience`, `gpuAvailable`, `hardwareAccess` - Connect to Neon PostgreSQL via `process.env.DATABASE_URL` - CORS configured for same origin (not needed but safe) - Health check endpoint at `GET /api/health` **2. Vercel Configuration Requirements:** - **Root `vercel.json`:** Rewrite `/api/auth/*` to serverless function, build command `npm run build:all` - **Backend `vercel.json`:** Configure as serverless function with 1024MB memory - **Root `package.json`:** Add scripts: `"install:all"`, `"build:all"`, `"dev:all"` using `concurrently` **3. Frontend Integration:** - `AuthProvider` wraps entire app in `src/theme/Layout/index.tsx` - `SignupForm` includes all required background questions as form fields - `TranslationButton` component checks `useAuth()` and only renders if user logged in **4. Environment Variables (For Vercel Dashboard):** - `DATABASE_URL`: 'postgresql://neondb_owner:npg_Zw9ahkT5rAMF@ep-plain-star-ah8pr9jx-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require' - `AUTH_SECRET`: EN7xV3ym4JpPu9WRuUpNgQYxJ7fCqvIO+BBq+Kjc6MQ= - `FRONTEND_URL`: https://ai-driven-textbook.vercel.app
labels: ["authentication", "specification", "monorepo", "docusaurus", "vercel"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-auth-integration/spec.md
 - specs/002-auth-integration/checklists/requirements.md
 - history/prompts/002-auth-integration/2-auth-integration-spec.spec.prompt.md
tests:
 - none
---

## Prompt

/sp.specify Create detailed specification based on the monorepo constitution. **Project: All-in-One Auth Integration for Vercel** **1. Backend Server Requirements (`/auth-backend/server.ts`):** - Use Express + Better Auth with `@better-auth/express` - Custom user fields: `experienceLevel`, `programmingLanguages`, `roboticsExperience`, `gpuAvailable`, `hardwareAccess` - Connect to Neon PostgreSQL via `process.env.DATABASE_URL` - CORS configured for same origin (not needed but safe) - Health check endpoint at `GET /api/health` **2. Vercel Configuration Requirements:** - **Root `vercel.json`:** Rewrite `/api/auth/*` to serverless function, build command `npm run build:all` - **Backend `vercel.json`:** Configure as serverless function with 1024MB memory - **Root `package.json`:** Add scripts: `"install:all"`, `"build:all"`, `"dev:all"` using `concurrently` **3. Frontend Integration:** - `AuthProvider` wraps entire app in `src/theme/Layout/index.tsx` - `SignupForm` includes all required background questions as form fields - `TranslationButton` component checks `useAuth()` and only renders if user logged in **4. Environment Variables (For Vercel Dashboard):** - `DATABASE_URL`: 'postgresql://neondb_owner:npg_Zw9ahkT5rAMF@ep-plain-star-ah8pr9jx-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require' - `AUTH_SECRET`: EN7xV3ym4JpPu9WRuUpNgQYxJ7fCqvIO+BBq+Kjc6MQ= - `FRONTEND_URL`: https://ai-driven-textbook.vercel.app

## Response snapshot

Created a comprehensive feature specification for the All-in-One Auth Integration for Vercel, including user scenarios, functional requirements, and success criteria. The specification covers user registration with background information, login functionality, conditional access to the translation feature, and backend health monitoring. The specification is aligned with the project's monorepo constitution and includes all necessary components for the Better Auth integration.

## Outcome

- ✅ Impact: Created complete feature specification for auth integration following project constitution
- 🧪 Tests: none
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase with `/sp.plan`
- 🧠 Reflection: The specification successfully captures all requirements from the user input while maintaining focus on user value and business needs without implementation details.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin planning the implementation with `/sp.plan`
- Manual verification: Specification was validated against all quality criteria and is ready for planning