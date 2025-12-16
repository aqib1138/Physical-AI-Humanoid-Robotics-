---
id: 2
title: Update Book RAG Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: LENOVO
command: /sp.constitution
labels: [constitution, technical-book, rag, docusaurus, chunking, ingestion]
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

Updated the project constitution to enhance requirements for the technical book with RAG chatbot project. The updated constitution now includes eight core principles:

1. Spec-First Development - Enhanced with stricter validation requirements
2. Quality and Compliance - Enhanced with automated validation requirements
3. Test-First Documentation - Documentation as specification and test artifact
4. RAG Context Integrity - Enhanced with explicit context boundary requirements
5. Reproducible Deployments - With automated testing requirements
6. Modular Architecture - With versioned interfaces requirement
7. Explicit Chunking Strategy - New principle for semantic coherence in chunks
8. Automatic Content Ingestion - New principle for automated pipeline requirements

The constitution also includes Tech Stack Requirements, Development Workflow, and Governance sections, all aligned with the project requirements. Version updated from 1.0.0 to 1.1.0 to reflect the enhancements.

## Outcome

- ✅ Impact: Updated constitution with enhanced requirements and new principles for chunking and ingestion
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with enhanced constitution
- 🔁 Next prompts: Continue with spec creation using /sp.specify
- 🧠 Reflection: Constitution now fully reflects all RAG requirements including explicit chunking strategy and automatic content ingestion

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification with detailed requirements