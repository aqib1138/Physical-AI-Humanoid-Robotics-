---
id: "10"
title: "Update-constitution"
stage: "constitution"
date: "2025-12-16"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "master"
user: "LENOVO"
command: "/sp.constitution AI-native technical author and software architect using Claude Code + Spec-Kit Plus.

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

All specs validated, no TODOs"
labels: ["constitution", "spec-first", "ai-native", "technical-authoring", "docusaurus", "rag"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - .specify/memory/constitution.md
tests:
  - "Constitution validation"
  - "Principle alignment check"
  - "Version bump verification"
---

## Prompt

/sp.constitution AI-native technical author and software architect using Claude Code + Spec-Kit Plus.

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

## Response snapshot

Successfully updated the project constitution to version 1.4.0 with expanded governance for AI-native technical authoring. Key changes include:

1. Added Principle XII: AI-Native Technical Authoring Standards that specifically addresses Claude Code + Spec-Kit Plus workflows
2. Enhanced existing principles to reflect AI-native authoring practices:
   - Expanded Principle I (Spec-First Development) with AI-native authoring context
   - Enhanced Principle II (Quality and Compliance) with AI-native verification requirements
   - Updated Principle IX (Documentation Standards) for AI-native technical authoring
3. Maintained all original requirements for spec-first development, zero hallucinations, academic standards (Flesch-Kincaid grade 10-12), 50%+ peer-reviewed sources, and RAG context integrity
4. Preserved the mandatory tech stack requirements (Spec-Kit Plus, Claude Code, Docusaurus, GitHub Pages for book; OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud for RAG)
5. Updated the Sync Impact Report to reflect the version change and modifications made

The updated constitution now properly governs AI-native technical authoring workflows while maintaining all original quality and compliance requirements.

## Outcome

- ✅ Impact: Constitution updated to v1.4.0 with AI-native authoring governance
- 🧪 Tests: All principles validated and aligned with requirements
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive changes
- 🔁 Next prompts: Ready for continued development with new governance
- 🧠 Reflection: Constitution now properly supports AI-native authoring while maintaining quality standards

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements met
- Graders run and results (PASS/FAIL): PASS - constitution update successful
- Prompt variant (if applicable): Standard constitution update with AI-native focus
- Next experiment (smallest change to try): Continue development with updated governance