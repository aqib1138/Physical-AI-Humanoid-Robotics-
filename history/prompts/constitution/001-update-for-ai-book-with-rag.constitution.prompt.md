---
id: 1
title: "Update constitution for AI/Spec-Driven Technical Book with RAG"
stage: constitution
date: "2025-12-16"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "master"
user: "LENOVO"
command: "/sp.constitution"
labels: ["constitution", "spec-driven", "rag", "docusaurus"]
link_spec: null
link_ticket: null
link_adr: null
link_pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Prompt

```
Project: AI/Spec-Driven Technical Book with Embedded RAG Chatbot Objectives


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

# Response

Updated the project constitution to properly reflect all requirements for the AI/Spec-Driven Technical Book with Embedded RAG Chatbot project. Key changes made:

1. Updated the project title to "AI/Spec-Driven Technical Book with Embedded RAG Chatbot Constitution"
2. Enhanced Principle I (Spec-First Development) with explicit zero hallucinations requirement
3. Renamed Principle II to "Quality and Compliance Authoring Rules" and enhanced with specific authoring rules
4. Enhanced Principle IV (RAG Context Integrity) with explicit two modes requirement
5. Added new Principle IX (Documentation Standards) for specifications and reproducible deployment steps
6. Added new Principle X (Success Criteria) with specific success metrics
7. Updated version from 1.1.0 to 1.2.0 with appropriate sync impact report
8. Ensured all requirements from the user input are properly reflected in the constitution

The updated constitution now fully aligns with the project requirements including the tech stack mandates, RAG requirements, authoring rules, and success criteria.

# Outcome

The constitution has been successfully updated to version 1.2.0 and reflects all specified requirements for the AI/Spec-Driven Technical Book with Embedded RAG Chatbot project. The document now properly governs development practices for the technical book project with all specified constraints and requirements.

# Evaluation

The updated constitution properly captures all requirements from the user input:
- ✅ Spec-first development with zero hallucinations
- ✅ Authoring rules (Flesch-Kincaid grade 10-12, 50%+ peer-reviewed sources, 0% plagiarism)
- ✅ Tech stack requirements (Docusaurus, GitHub Pages, OpenAI Agents/ChatKit, FastAPI, Neon, Qdrant)
- ✅ RAG requirements (automatic ingestion, explicit chunking, two modes, no answers outside context)
- ✅ Documentation and reproducible deployment requirements
- ✅ Success criteria (published book, working RAG chatbot, validated specs)