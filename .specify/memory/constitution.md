<!-- SYNC IMPACT REPORT:
Version change: 1.3.0 → 1.4.0 (expanded governance for AI-native authoring)
Modified principles: I. Spec-First Development (expanded for AI-native authoring), II. Quality and Compliance Authoring Rules (expanded for AI-native authoring), IX. Documentation Standards (expanded for AI-native technical authoring)
Added sections: XII. AI-Native Technical Authoring Standards (Claude Code + Spec-Kit Plus authoring rules)
Removed sections: N/A
Templates requiring updates: .specify/templates/plan-template.md (✅ updated), .specify/templates/spec-template.md (✅ updated), .specify/templates/tasks-template.md (✅ updated)
Follow-up TODOs: None
-->
# AI-Native Technical Book Authoring Constitution: Docusaurus + RAG Chatbot

## Core Principles

### I. Spec-First Development
All development follows a spec-driven approach with zero hallucinations. Every feature must begin with a comprehensive specification that includes requirements, acceptance criteria, and validation methods before any implementation begins. No code shall be written without an approved specification that has been validated against the project's architectural constraints. As an AI-native technical author and software architect using Claude Code + Spec-Kit Plus, all specifications must be precise, testable, and source-backed to ensure verifiability and eliminate hallucinations.

### II. Quality and Compliance Authoring Rules
All content must meet academic standards (Flesch-Kincaid grade 10-12), with 50%+ peer-reviewed or official sources, and 0% plagiarism. Every claim must be source-backed with APA-style citations, ensuring credibility and verifiability. Content must undergo automated plagiarism detection and readability scoring before acceptance. Zero hallucinations and no fabrication of information is allowed. As an AI-native technical author, all claims must be source-backed with verifiable references and validated against official documentation.

### III. Test-First Documentation
All features and components must be documented and validated before implementation. Documentation serves as both specification and test artifact, with examples and expected behaviors clearly defined. All documentation must include reproducible examples and validation procedures to ensure correctness. Documentation must be structured to enable automatic ingestion into the RAG system with clear test scenarios.

### IV. RAG Context Integrity
The RAG system must only provide answers based on retrieved context, with no fabricated responses or hallucinations. The system implements two modes: Full book Q&A and Selected-text-only Q&A, ensuring responses stay within retrieved context boundaries. The system must explicitly indicate when answers cannot be provided due to insufficient context. No answers outside retrieved context are permitted. Content must be structured with explicit chunking strategy for effective retrieval.

### V. Reproducible Deployments
All deployments to GitHub Pages must be reproducible with documented steps. Every component, configuration, and deployment process must be version-controlled and replicable across environments with clear, step-by-step procedures. Deployment pipelines must include automated testing and validation steps. Reproducible code and deployment steps are mandatory for all components.

### VI. Modular Architecture
Components (book, RAG, chatbot) should be modular and independently deployable. This ensures maintainability, scalability, and the ability to update individual components without affecting the entire system. Module interfaces must be well-defined and versioned to prevent breaking changes. Architecture must support the integration of Claude Code and Spec-Kit Plus workflows.

### VII. Explicit Chunking Strategy
Book content must be ingested using an explicit chunking strategy that preserves semantic coherence while enabling effective retrieval. Chunks must be designed to be contextually complete and appropriately sized for the RAG system's token limitations. The chunking strategy must be documented and validated for retrieval effectiveness. Chunks must maintain technical accuracy and completeness for AI-native authoring.

### VIII. Automatic Content Ingestion
The system must automatically ingest book content as it is updated, with minimal manual intervention. Content ingestion pipeline must include validation, chunking, embedding, and indexing steps. The pipeline must provide monitoring and alerting for failures. The system must handle Docusaurus-generated content and integrate with Claude Code workflows.

### IX. Documentation Standards
Complete specifications for book, RAG pipeline, schemas, and APIs must be provided. All code must be reproducible with clear deployment steps documented. Documentation must be comprehensive enough to enable independent reproduction of the entire system by third parties. For AI-native technical authoring, documentation must include Claude Code workflows, Spec-Kit Plus integration, and technical authoring guidelines. Technical documentation must meet academic standards with proper citations and verifiable claims.

### X. Success Criteria
The project achieves success when: (1) the book is published on GitHub Pages, (2) an embedded, working RAG chatbot is operational, (3) all specifications are validated with no remaining TODOs, and (4) the Spec → Build → Verify → Publish cycle is completed successfully. The RAG chatbot must support both Full book Q&A and Selected-text-only Q&A modes without generating responses outside the retrieved context.

### XI. AI/Robotics Content Standards
All AI and robotics content, particularly for NVIDIA Isaac-based systems, must adhere to specialized standards: (1) Perception system documentation must include synthetic data generation techniques and training model validation procedures, (2) Navigation content must cover path planning algorithms and humanoid-specific mobility challenges, (3) Training content must address simulation-to-reality transfer and hardware-accelerated processing considerations, (4) Integration content must detail ROS 2 compatibility and pipeline optimization for real-time performance.

### XII. AI-Native Technical Authoring Standards
As an AI-native technical author and software architect using Claude Code + Spec-Kit Plus, all work must follow: (1) Claude Code workflows for AI-assisted development and specification generation, (2) Spec-Kit Plus integration for spec-driven development, (3) AI-native authoring practices that leverage Claude's capabilities while maintaining human oversight, (4) Technical writing standards that ensure content is accurate, verifiable, and appropriate for target audiences (AI engineers and CS students), (5) Integration of AI tools with human expertise for optimal technical documentation quality.

## Tech Stack Requirements

Technology stack mandates specific tools: Book development using Spec-Kit Plus, Claude Code, Docusaurus, deployed to GitHub Pages. RAG system must use OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, and Qdrant Cloud (Free tier). For AI/robotics content, NVIDIA Isaac Sim, Isaac ROS, and Nav2 integration tools must be documented and validated. Deviations require architectural review and approval. All technology selections must support the project's requirements for reproducibility and long-term maintenance. The stack must enable AI-native technical authoring workflows.

## Development Workflow

Follow the Spec → Build → Verify → Publish cycle for all deliverables. Each phase requires completion validation before advancing. Specifications must be validated, builds must pass all tests, verification must confirm requirements are met, and publishing must result in accessible, working functionality. Each phase must produce artifacts that can be independently verified and validated. For AI-native authoring, validation must include Claude Code integration and Spec-Kit Plus workflow verification. For AI/robotics modules, validation must include simulation-to-reality transfer testing and hardware integration verification.

## Governance

This constitution governs all development practices for the technical book project. All team members must verify compliance with these principles during reviews. Changes to the constitution require documentation of rationale, approval from project leadership, and a migration plan for existing code. The constitution supersedes all other practices and guidelines. Regular compliance reviews must be conducted to ensure adherence to these principles. AI-native authoring workflows must be regularly reviewed for compliance with spec-first and zero-hallucination requirements.

**Version**: 1.4.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16