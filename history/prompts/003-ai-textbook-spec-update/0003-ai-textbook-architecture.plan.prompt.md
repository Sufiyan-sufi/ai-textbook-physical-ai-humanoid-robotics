---
id: 0003
title: AI Textbook Architecture
stage: plan
date: 2025-12-09
surface: agent
model: Claude Sonnet 4.5
feature: 003-ai-textbook-spec-update
branch: 003-ai-textbook-spec-update
user:
command: /sp.plan
labels: ["architecture","planning","ai-textbook","robotics","docusaurus","fastapi","rag"]
links:
  spec: /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/plan.md
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/research.md
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/data-model.md
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/quickstart.md
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/contracts/textbook-api.yaml
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/contracts/ai-tutor-api.yaml
 - /mnt/d/hackathon-book-2025/specs/003-ai-textbook-spec-update/contracts/simulation-api.yaml
 - /mnt/d/hackathon-book-2025/CLAUDE.md
tests:
 -
---

## Prompt

# ADR-001: AI-Native Textbook Architecture

## Status
Accepted

## Decision
Use modular architecture:
- Docusaurus (book frontend)
- FastAPI (backend: RAG + ChatKit)
- ChatKit/OpenAI Agents (intelligent tutoring + actions)
- Neon Postgres + Qdrant (memory + vector search)

## Rationale
- Clear separation of content, backend logic, and agent intelligence
- Enables personalization, translation, and RAG retrieval
- Supports simulation tools (Gazebo/Isaac/Unity) via API endpoints
- Scales cleanly for future robotics textbooks

## Alternatives Considered
1. Monolithic full-stack app
2. Static Docusaurus pages without backend
3. Backend-only platform using templating

## Consequences
Positive:
- Scalable, maintainable, AI-native textbook
- Fast iteration using Claude Code + Spec-Kit Plus
Negative:
- Requires multi-service deployment
Constraints:
- Must preserve API contracts
- Must provide hooks for RAG, personalization, translation

## Response snapshot

Implementation plan created for the AI-Native Textbook on Physical AI & Humanoid Robotics. The plan includes:

1. **Technical Context**: Defined languages (Python 3.11, JS/TS), dependencies (Docusaurus, FastAPI, ChatKit, Neon Postgres, Qdrant, ROS 2), and platform requirements (Ubuntu 22.04, Jetson/RTX).

2. **Constitution Check**: Both pre-design and post-design checks performed, confirming compliance with all core principles from the project constitution.

3. **Project Structure**: Detailed architecture with separate backend (FastAPI) and frontend (Docusaurus) services, plus simulation components.

4. **Research**: Completed research on architecture decisions, technology integration, and best practices.

5. **Data Model**: Defined core entities including Chapter, Tutorial, User, UserProgress, SimulationSession, AIChatSession, and ContentVector.

6. **API Contracts**: Created OpenAPI specifications for textbook, AI tutor, and simulation APIs.

7. **Quickstart Guide**: Comprehensive setup and development workflow documentation.

8. **Agent Context Update**: Updated Claude Code context with project-specific technologies.

## Outcome

- ✅ Impact: Complete implementation plan with architecture, data models, API contracts, and documentation
- 🧪 Tests: No tests required at planning stage
- 📁 Files: Multiple planning artifacts created including plan.md, research.md, data-model.md, quickstart.md, and API contracts
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Planning phase completed successfully with all Phase 0 and Phase 1 deliverables

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks with /sp.tasks
