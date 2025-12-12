---
id: 0001
title: Textbook Physical AI Robotics Plan
stage: plan
date: 2025-12-08
surface: agent
model: Claude Sonnet 4.5
feature: 002-physical-ai-robotics-book
branch: 002-physical-ai-robotics-book
user:
command: /sp.plan
labels: ["architecture","modular","docusaurus","fastapi","chatkit","robotics","textbook"]
links:
  spec: /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/plan.md
 - /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/research.md
 - /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/data-model.md
 - /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/quickstart.md
 - /mnt/d/hackathon-book-2025/specs/002-physical-ai-robotics-book/contracts/textbook-api.yaml
 - /mnt/d/hackathon-book-2025/CLAUDE.md
tests:
 -
---

## Prompt

Context
The textbook on Physical AI & Humanoid Robotics requires an AI-native, interactive publishing platform.
Three core components must be integrated:
1.    Docusaurus – for documentation-style textbook publishing, versioning, and rich educational UI.
2.    FastAPI backend – to serve the interactive chatbot, run robotics demos, handle API requests, and integrate simulation endpoints.
3.    OpenAI ChatKit – as the agentic AI layer providing contextual tutoring, interactive explanations, and code-generation assistance.
We must decide how these components will interact and which architecture best supports long-term maintainability, extensibility, and AI-native learning.
Two primary architectural approaches exist:
•    Monolithic Learning Platform (single combined codebase)
•    Modular AI-Native Documentation System (Docusaurus frontend, FastAPI backend, ChatKit as agent layer)
Decision
We will use a modular architecture:
•    Docusaurus for the textbook frontend and navigation
•    FastAPI as a standalone backend microservice
•    ChatKit integrated as an agentic layer connecting both frontend and backend
This ensures a clear separation between content, backend logic, and AI reasoning.
Alternatives Considered
1. Monolithic Learning Platform (Single Codebase)
•    Pro:
o    Simplified deployment pipeline
o    Single hosting target
o    Tight integration between UI, backend, and AI
•    Con:
o    Poor scalability
o    Harder to update each subsystem independently
o    AI agent layer becomes tightly coupled to the documentation engine
o    Complex CI/CD due to mixed responsibilities
2. Modular Docusaurus + FastAPI + ChatKit (Chosen Architecture)
•    Pro:
o    Clean separation of responsibilities
o    Docusaurus stays solely for documentation and UI
o    FastAPI handles backend logic, simulations, sandboxing, and robotics APIs
o    ChatKit acts as an independent agent interface connecting both layers
o    Easier upgrades (ROS 2 APIs, new agent tools, new content versions)
o    Ideal for educational, scalable, AI-native textbooks
•    Con:
o    Requires multi-service deployment strategy
o    More infrastructure overhead
3. Backend-Centric Platform (FastAPI with templated frontend)
•    Pro:
o    Backend stays the "brain"
o    Easy to extend services for robotics simulation, Isaac endpoints
o    Minimal external dependencies
•    Con:
o    Poor documentation experience for a full textbook
o    Lack of rich navigation, versioning, search, and book-like UX
o    Higher development effort to match Docusaurus features
Rationale
A modular architecture ensures:
•    Docusaurus provides best-in-class documentation UX, MDX support, versioning, sidebar nav, and publishing simplicity.
•    FastAPI supports robotics simulation endpoints, code execution hooks, real-time interaction, and custom APIs.
•    ChatKit enhances the textbook with an agentic AI tutor, enabling live explanations, code corrections, and ROS/Gazebo/Isaac assistance inside the book.
This architecture ensures future extensibility toward:
•    Live robot control demos
•    Embedded simulations
•    Reinforcement-learning experiment walkthroughs
•    Isaac/Unity training pipelines
The modular approach allows each piece to evolve independently without breaking the entire system.
Consequences
Positive:
•    Scalable, future-proof architecture
•    Easy to update Docusaurus content without touching backend
•    FastAPI can evolve into a robotics microservice hub
•    ChatKit agents add adaptive teaching, code generation, and interactive labs
•    Clean abstraction layers enable long-term maintainability
Negative:
•    Requires multi-service deployment and CI/CD setup
•    Increased initial infrastructure complexity
•    Cross-service communication requires careful API design
Constraints:
•    Must maintain API contract stability between Docusaurus and FastAPI
•    ChatKit agent must have clear tool definitions for robotics simulations
•    Documentation and backend must version-lock with the textbook content
•    Ensure all components remain open-source friendly and replicable

## Response snapshot

# Implementation Plan: Textbook on Physical AI & Humanoid Robotics

**Branch**: `002-physical-ai-robotics-book` | **Date**: 2025-12-08 | **Spec**: [link]
**Input**: Feature specification from `/specs/002-physical-ai-robotics-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The textbook on Physical AI & Humanoid Robotics requires an AI-native, interactive publishing platform. Three core components must be integrated:
1.    Docusaurus – for documentation-style textbook publishing, versioning, and rich educational UI.
2.    FastAPI backend – to serve the interactive chatbot, run robotics demos, handle API requests, and integrate simulation endpoints.
3.    OpenAI ChatKit – as the agentic AI layer providing contextual tutoring, interactive explanations, and code-generation assistance.

The architecture will use a modular approach with Docusaurus for the textbook frontend and navigation, FastAPI as a standalone backend microservice, and ChatKit integrated as an agentic layer connecting both frontend and backend. This ensures a clear separation between content, backend logic, and AI reasoning.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript (Node.js 18+)
**Primary Dependencies**: Docusaurus, FastAPI, OpenAI ChatKit, ROS 2 (Humble Hawksbill), Gazebo, NVIDIA Isaac Sim, Unity (ML-Agents)
**Storage**: Git-based version control, potential cloud storage for simulation assets
**Testing**: pytest for backend, Jest for frontend, simulation validation tests
**Target Platform**: Web-based (Docusaurus), Linux server (ROS 2/Gazebo/Isaac), cross-platform for Unity
**Project Type**: Web application with interactive backend services
**Performance Goals**: <500ms response time for AI interactions, <2s page load for documentation
**Constraints**: Must maintain API contract stability between Docusaurus and FastAPI, ChatKit agent must have clear tool definitions for robotics simulations, documentation and backend must version-lock with textbook content
**Scale/Scope**: Educational platform for students, educators, and researchers; single textbook with interactive features

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Gates determined based on constitution file:
- Scientific Accuracy: All robotics algorithms and concepts must be verified from authoritative sources - **PASSED**: Research confirms use of peer-reviewed sources and technical documentation
- Conceptual Clarity: Content must be suitable for engineering students and robotics researchers - **PASSED**: Multi-level approach with progressive scaffolding addresses this requirement
- Hands-on Practicality: All examples and tutorials must be reproducible and executable - **PASSED**: Multi-simulation environment integration ensures practical, executable examples
- Technical Rigor: Follow standards from robotics, mechatronics, AI safety, and embodied intelligence - **PASSED**: Architecture uses established frameworks (ROS 2, Gazebo, Isaac Sim) following industry standards
- Progressive Scaffolding: Concepts introduced in beginner-friendly form, then expanded into advanced details - **PASSED**: Chapter structure includes learning outcomes and progressive complexity
- Safety & Ethics Alignment: Content must align with modern AI safety and human-robot interaction guidelines - **PASSED**: AI tutor component will include safety guidelines and ethical considerations
- Documentation-First Writing: All systems, modules, and algorithms must be reproducible and implementable - **PASSED**: Docusaurus-based documentation with executable examples ensures reproducibility

## Project Structure

### Documentation (this feature)

```text
specs/002-physical-ai-robotics-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

my-website/
├── docs/
│   ├── chapters/
│   └── tutorials/
├── src/
│   ├── components/
│   └── pages/
├── static/
└── docusaurus.config.js

.simulate/
├── ros2_ws/
├── gazebo/
├── unity/
└── isaac/
```

**Structure Decision**: Web application with interactive backend services. The Docusaurus-based frontend serves the textbook content with interactive elements, while the FastAPI backend handles robotics simulations, API requests, and AI interactions. Simulation environments are organized in the .simulate directory for ROS 2, Gazebo, Unity, and Isaac integration.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Prompt

Context
The textbook on Physical AI & Humanoid Robotics requires an AI-native, interactive publishing platform.
Three core components must be integrated:
1.    Docusaurus – for documentation-style textbook publishing, versioning, and rich educational UI.
2.    FastAPI backend – to serve the interactive chatbot, run robotics demos, handle API requests, and integrate simulation endpoints.
3.    OpenAI ChatKit – as the agentic AI layer providing contextual tutoring, interactive explanations, and code-generation assistance.
We must decide how these components will interact and which architecture best supports long-term maintainability, extensibility, and AI-native learning.
Two primary architectural approaches exist:
•    Monolithic Learning Platform (single combined codebase)
•    Modular AI-Native Documentation System (Docusaurus frontend, FastAPI backend, ChatKit as agent layer)
Decision
We will use a modular architecture:
•    Docusaurus for the textbook frontend and navigation
•    FastAPI as a standalone backend microservice
•    ChatKit integrated as an agentic layer connecting both frontend and backend
This ensures a clear separation between content, backend logic, and AI reasoning.
Alternatives Considered
1. Monolithic Learning Platform (Single Codebase)
•    Pro:
o    Simplified deployment pipeline
o    Single hosting target
o    Tight integration between UI, backend, and AI
•    Con:
o    Poor scalability
o    Harder to update each subsystem independently
o    AI agent layer becomes tightly coupled to the documentation engine
o    Complex CI/CD due to mixed responsibilities
2. Modular Docusaurus + FastAPI + ChatKit (Chosen Architecture)
•    Pro:
o    Clean separation of responsibilities
o    Docusaurus stays solely for documentation and UI
o    FastAPI handles backend logic, simulations, sandboxing, and robotics APIs
o    ChatKit acts as an independent agent interface connecting both layers
o    Easier upgrades (ROS 2 APIs, new agent tools, new content versions)
o    Ideal for educational, scalable, AI-native textbooks
•    Con:
o    Requires multi-service deployment strategy
o    More infrastructure overhead
3. Backend-Centric Platform (FastAPI with templated frontend)
•    Pro:
o    Backend stays the "brain"
o    Easy to extend services for robotics simulation, Isaac endpoints
o    Minimal external dependencies
•    Con:
o    Poor documentation experience for a full textbook
o    Lack of rich navigation, versioning, search, and book-like UX
o    Higher development effort to match Docusaurus features
Rationale
A modular architecture ensures:
•    Docusaurus provides best-in-class documentation UX, MDX support, versioning, sidebar nav, and publishing simplicity.
•    FastAPI supports robotics simulation endpoints, code execution hooks, real-time interaction, and custom APIs.
•    ChatKit enhances the textbook with an agentic AI tutor, enabling live explanations, code corrections, and ROS/Gazebo/Isaac assistance inside the book.
This architecture ensures future extensibility toward:
•    Live robot control demos
•    Embedded simulations
•    Reinforcement-learning experiment walkthroughs
•    Isaac/Unity training pipelines
The modular approach allows each piece to evolve independently without breaking the entire system.
Consequences
Positive:
•    Scalable, future-proof architecture
•    Easy to update Docusaurus content without touching backend
•    FastAPI can evolve into a robotics microservice hub
•    ChatKit agents add adaptive teaching, code generation, and interactive labs
•    Clean abstraction layers enable long-term maintainability
Negative:
•    Requires multi-service deployment and CI/CD setup
•    Increased initial infrastructure complexity
•    Cross-service communication requires careful API design
Constraints:
•    Must maintain API contract stability between Docusaurus and FastAPI
•    ChatKit agent must have clear tool definitions for robotics simulations
•    Documentation and backend must version-lock with the textbook content
•    Ensure all components remain open-source friendly and replicable

## Outcome

- ✅ Impact:
- 🧪 Tests:
- 📁 Files:
- 🔁 Next prompts:
- 🧠 Reflection:

## Evaluation notes (flywheel)

- Failure modes observed:
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try):
