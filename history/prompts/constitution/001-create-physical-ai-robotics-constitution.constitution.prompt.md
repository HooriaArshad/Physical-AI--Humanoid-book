---
id: 001
title: Create Physical AI Robotics Constitution
stage: constitution
date: 2025-12-26
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "initialization", "project-setup", "principles"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

Project Name: Physical AI & Humanoid Robotics — Essentials

Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.

Scope:
- 6 short chapters:
  1. Introduction to Physical AI
  2. Basics of Humanoid Robotics
  3. ROS 2 Fundamentals
  4. Digital Twin Simulation (Gazebo + Isaac)
  5. Vision-Language-Action Systems
  6. Capstone: Simple AI-Robot Pipeline
- Clean UI
- Free-tier friendly
- Lightweight embeddings

Core Principles:
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text

Key Features:
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

Constraints:
- No heavy GPU usage
- Minimal embeddings

Success Criteria:
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

## Response snapshot

Created comprehensive constitution v1.0.0 for Physical AI & Humanoid Robotics textbook project.

**7 Core Principles Established:**
1. Simplicity First - Minimal viable implementation, single purpose components
2. Accuracy and Quality - Technical correctness, tested code examples
3. Minimalism in Scope - 6 chapters, 15-30 min read time each
4. Free-Tier Architecture (NON-NEGOTIABLE) - Qdrant/Neon/GitHub Pages free tiers
5. Fast Builds and Deployment - <2 min builds, <5 min CI/CD
6. RAG Integrity - Answers only from textbook, no external knowledge
7. Testing and Validation - Build success, retrieval accuracy, accessibility

**Technical Stack Locked:**
- Frontend: Docusaurus (React)
- Vector: Qdrant Cloud (free tier)
- DB: Neon PostgreSQL (free tier)
- Backend: FastAPI (Python 3.11+)
- Embeddings: sentence-transformers (CPU, <100MB)
- Host: GitHub Pages

**Performance Budgets:**
- Embedding: <500ms/query (CPU)
- RAG latency: <2s end-to-end
- Page load: <3s on 3G
- Vector search: <200ms

**Development Workflows:**
- Content creation (write → review → test → build → commit)
- Feature development (spec → plan → implement → validate → document)
- RAG pipeline (extract → embed → store → test → deploy)
- Deployment (build → test → push → deploy → validate)

**Governance:**
- Semantic versioning for constitution amendments
- ADRs required for architectural decisions
- Complexity must be justified in plan.md
- All PRs verify constitution compliance

## Outcome

- ✅ Impact: Project constitution established, defining all core principles, technical constraints, and workflows for the AI-Native textbook
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: .specify/memory/constitution.md (created)
- 🔁 Next prompts: `/sp.specify` to create feature specifications based on constitution, or begin content creation for chapters
- 🧠 Reflection: Constitution emphasizes free-tier constraints (Qdrant, Neon, GitHub Pages), RAG integrity (textbook-only answers), and educational quality (accuracy, simplicity). Clear technical stack and performance budgets prevent scope creep.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual review - PASS (all placeholders filled, principles clear and testable)
- Prompt variant (if applicable): baseline
- Next experiment (smallest change to try): Validate constitution principles against actual Docusaurus setup and RAG implementation constraints
