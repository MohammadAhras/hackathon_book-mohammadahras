---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: qmobx
command: /sp.constitution
labels: ["constitution","setup","project-initialization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Spec–Driven Technical Book with Embedded RAG Chatbot

Purpose:
Author and deploy a technical book using Spec-Kit Plus and Claude Code with Docusaurus on GitHub Pages, including a tightly grounded RAG chatbot that answers questions strictly from the book content or user-selected text.

Core principles:
- Specification-first execution
- Technical accuracy and verifiability
- Developer-focused clarity
- Reproducibility
- Zero-hallucination AI behavior

Book standards:
- Format: Docusaurus (MD/MDX)
- Deployment: GitHub Pages
- Audience: Intermediate–advanced developers
- Structure: Architecture, tooling, authoring workflow, RAG design, backend, frontend embed, testing
- Readability: Flesch-Kincaid grade 10–12
- All examples executable or deployable

RAG standards:
- Retrieval strictly from indexed book content
- User-selected text overrides global retrieval
- Mandatory source section citation
- Deterministic, grounded responses

Technical constraints:
- Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud (Free Tier)
- AI: OpenAI Agents SDK / ChatKit SDK
- Embeddings, chunking, schemas fully documented

Quality validation:
- No broken builds, links, or code
- Hallucination and retrieval accuracy tests
- Deployed chatbot works on GitHub Pages

Decisions to document:
- Chunking strategy
- Embedding model
- RAG architecture
- Prompt design
- Security boundaries

Success criteria:
- Successful book deployment
- Reproducible setup
- RAG chatbot answers only from source content
- Selection-scoped answers enforced
- All validation tests pass
```

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the specific details for the AI-Spec–Driven Technical Book with Embedded RAG Chatbot project. The constitution now includes six core principles aligned with the project requirements: Specification-First Execution, Technical Accuracy and Verifiability, Developer-Focused Clarity, Reproducibility, Zero-Hallucination AI Behavior, and Grounded RAG Responses. Additional sections cover Book Standards, RAG Standards and Technical Constraints, and Quality Validation requirements. The governance section establishes the authority of the constitution over all project activities.

## Outcome

- ✅ Impact: Created project constitution with 6 core principles and specific standards for technical book and RAG chatbot
- 🧪 Tests: None needed for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with project-specific values
- 🔁 Next prompts: Plan the technical book architecture and RAG implementation
- 🧠 Reflection: Constitution provides clear governance framework for project compliance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin spec creation for the technical book