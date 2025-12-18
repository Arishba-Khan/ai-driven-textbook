---
id: 005
title: Debug Connection Failure
stage: misc
date: 2025-12-18
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: 122
command: browser_subagent
labels: [debugging, connection-refused, vercel, huggingface]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - null
tests:
 - browser_console_logs
---

## Prompt

backend server is not working. 

## Response snapshot

I investigated the live site at `https://ai-driven-textbook.vercel.app` and found that the chatbot is still trying to connect to `http://localhost:8000/chat/stream`. This is causing the `net::ERR_CONNECTION_REFUSED` error. You must commit and push the changes I made locally to your GitHub repository so Vercel can deploy the correct Hugging Face URL.

## Outcome

- ✅ Impact: Identified the root cause of the connection failure as stale frontend code.
- 🧪 Tests: Verified with browser subagent console logs.
- 📁 Files: null
- 🧠 Reflection: Live site inspection via browser subagent is highly effective for diagnosing deployment mismatches.
