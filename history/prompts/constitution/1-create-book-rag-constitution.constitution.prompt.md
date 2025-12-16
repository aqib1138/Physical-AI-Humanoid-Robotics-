---
id: 1
title: Create Book RAG Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: LENOVO
command: /sp.constitution
labels: [constitution, technical-book, rag, docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests: []
---

## Prompt

```
Objectives

Write a spec-driven technical book using Docusaurus, deployed to GitHub Pages.

Build and embed a RAG chatbot inside the book.

Authoring Rules

Spec-first, zero hallucinations

All claims source-backed (APA style)

50%+ peer-reviewed / official sources

0% plagiarism

Flesch-Kincaid grade 10–12

Tech Stack (Mandatory)

Book: Spec-Kit Plus, Claude Code, Docusaurus, GitHub Pages
RAG: OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free)

RAG Requirements

Ingest book content automatically

Explicit chunking + embedding strategy

Two modes:

Full book Q&A

Selected-text-only Q&A

No answers outside retrieved context

Documentation

Specs for book, RAG pipeline, schemas, APIs

Reproducible code + deployment steps

Success Criteria

Published book on GitHub Pages

Embedded, working RAG chatbot

All specs validated, no TODOs

Mode: Spec → Build → Verify → Publish
```

## Response snapshot

Updated the project constitution to reflect requirements for the technical book with RAG chatbot project. The new constitution includes six core principles:

1. Spec-First Development - All development follows a spec-driven approach with zero hallucinations
2. Quality and Compliance - Content meets academic standards with source-backed claims
3. Test-First Documentation - All features must be documented and validated before implementation
4. RAG Context Integrity - System only provides answers based on retrieved context
5. Reproducible Deployments - All deployments must be reproducible with documented steps
6. Modular Architecture - Components should be modular and independently deployable

The constitution also includes Tech Stack Requirements, Development Workflow, and Governance sections, all aligned with the project requirements.

## Outcome

- ✅ Impact: Created initial project constitution for technical book with RAG chatbot
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with new constitution
- 🔁 Next prompts: Continue with spec creation using /sp.specify
- 🧠 Reflection: Constitution properly reflects all project requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification