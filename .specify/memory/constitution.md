# AI-Spec–Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Specification-First Execution
Specification-first execution: All development begins with a detailed specification that defines scope, interfaces, acceptance criteria, and validation tests; Code implementation follows the spec precisely; Changes to behavior require specification updates first.

### Technical Accuracy and Verifiability
Technical accuracy and verifiability: All content and code examples must be fact-checked and tested for correctness; Code examples must be executable or deployable as provided; Claims must be supported by evidence or references.

### Developer-Focused Clarity
Developer-focused clarity: Content must be accessible to intermediate-advanced developers; Explanations include practical examples and use cases; Complex concepts are broken down into digestible, progressive learning steps.

### Reproducibility
Reproducibility: All setup procedures, deployments, and examples must be reproducible by others following the documentation; Dependencies and environment requirements are explicitly documented; Step-by-step workflows ensure consistent outcomes.

### Zero-Hallucination AI Behavior
Zero-hallucination AI behavior: The RAG chatbot must only respond based on indexed book content or user-selected text; No generation of information outside the source material; Mandatory source citations for all responses.

### Grounded RAG Responses
Grounded responses: The chatbot retrieves strictly from indexed book content; User-selected text overrides global retrieval; Responses must be deterministic and consistently grounded in source material.

## Book Standards
Docusaurus-based technical book standards: Content formatted in MD/MDX for Docusaurus; Deployed to GitHub Pages; Target audience is intermediate-advanced developers; Content structured with architecture, tooling, workflow, and implementation details; Readability maintained at Flesch-Kincaid grade 10-12; All examples are executable or deployable.

## RAG Standards and Technical Constraints
RAG implementation standards: Strictly sourced from indexed book content; User selection overrides global retrieval; Mandatory source section citations; Deterministic, grounded responses. Technical stack: Backend with FastAPI; Database with Neon Serverless Postgres; Vector DB with Qdrant Cloud (Free Tier); AI integration with OpenAI Agents SDK/ChatKit SDK; Full documentation of embeddings, chunking, and schemas.

## Quality Validation
Quality validation requirements: No broken builds, links, or code examples; Comprehensive hallucination and retrieval accuracy tests; Deployed chatbot functions correctly on GitHub Pages; All examples pass execution tests; Content meets readability standards.

## Governance
This constitution governs all aspects of the AI-Spec–Driven Technical Book project. All development, documentation, and implementation activities must comply with these principles. Changes to this constitution require explicit approval and documentation of the rationale. The project team must regularly review compliance with these principles during development cycles.

**Version**: 1.0.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-22
