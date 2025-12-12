<!-- Sync Impact Report:
Version change: 1.0.0 → 2.0.0
List of modified principles:
- Scientific Accuracy → Technical Accuracy
- Conceptual Clarity → Clarity for Engineering Students
- Hands-on Practicality → Reproducibility
- Technical Rigor → Consistency Across Modules
- Progressive Scaffolding → Modularity
- Safety & Ethics Alignment → Safety Standards Compliance
- Documentation-First Writing → RAG-Friendly Documentation
Added sections: Key Standards (updated), Constraints (updated)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->
# AI-Native Textbook on Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical Accuracy
Technical accuracy through verification from authoritative primary sources (robotics research labs, IEEE, ACM, arXiv, DARPA, AI/ML academic literature). All robotics/AI claims must be verified with primary sources.

### Clarity for Engineering Students
Clarity for engineering students with focus on ROS 2, Gazebo, Isaac, Unity. Content must be accessible and clear for the target audience of engineering students, robotics researchers, and AI practitioners.

### Reproducibility
Reproducibility ensuring all code and simulations run as documented. All examples, tutorials, and simulations must be executable and reproducible on the target platform.

### Consistency Across Modules
Consistency across modules following the progression: ROS → Simulation → Isaac → VLA → Capstone. All content must maintain consistent terminology, approach, and quality standards across modules.

### Modularity
Modularity with chapters independently testable. Each chapter and module should be designed to function independently while contributing to the overall learning path.

### Safety Standards Compliance
Safety standards compliance with modern robotics and AI safety guidelines. All content must follow industry safety standards for robotics applications.

### RAG-Friendly Documentation
RAG-friendly documentation in clean Markdown optimized for retrieval. Content must be structured for effective retrieval-augmented generation and AI agent integration.

## Key Standards

*   All factual claims must be sourced, traceable, and cross-verified.
*   Citation format: APA (when citing research).
*   All diagrams and examples must be original.
*   All code must run on Ubuntu 22.04 with ROS 2 Humble.
*   Include simulation-ready URDF/SDF files where relevant.
*   RAG-friendly: Write in clean Markdown optimized for retrieval.
*   Source types:
    *   Minimum 50% peer-reviewed robotics + AI publications (IEEE RAS, ACM, Nature Robotics, Science Robotics).
    *   Remaining from credible sources: DARPA documents, textbooks, manufacturer technical manuals, white papers.
*   Code examples:
    *   Must be testable and executable in Claude Code.
    *   Use clear modular structures with explanations.
*   Diagrams and schematics:
    *   Must be described clearly so Claude Code can auto-generate them when needed.
*   Terminology consistency:
    *   Follow standard robotics vocabulary (ROS, kinematics, dynamics, actuation, sensors, RL, physical AI).

## Constraints

*   Book must be Docusaurus-compatible (MDX).
*   Must support ChatKit agent integration.
*   Must support localization and personalization hooks.
*   Must ensure reproducibility on Jetson + RTX workstation.
*   Avoid proprietary robot SDKs unless open documentation exists.
*   Total textbook length: 60,000–85,000 words.
*   Chapter count: 12–15 chapters, each 4,500–6,000 words.
*   Minimum 80 sources, with at least 40 peer-reviewed.
*   Format outputs:
    *   SpeckitPlus-compatible document export.
    *   PDF and EPUB versions at final stage.
*   Non-negotiable quality constraints:
    *   0% plagiarism tolerance.
    *   All robotics algorithms must be correct and tested.
    *   No hallucinated citations or formulas.

## Governance

*   Technical correctness: All robotics principles (kinematics, control, actuation, AI models) thoroughly validated.
*   Completeness: Covers Physical AI foundations → humanoid robotics → advanced autonomous embodied systems.
*   Academic acceptability: Ready for university undergraduate & graduate course adoption.
*   Practical utility: Engineers can build real robot modules using the book's instructions.
*   AI-native workflow: Claude Code and SpeckitPlus can seamlessly generate, execute, and update content.
*   Review readiness: Passes fact-checking, peer review, and plagiarism checks.
*   Coherence: Uniform writing style across all chapters, maintaining conceptual flow and terminology consistency.
*   Amendment procedure: Changes to this constitution require explicit review and approval with version increment.
*   Versioning policy: MAJOR for principle removals, MINOR for additions, PATCH for clarifications.
*   Compliance review: Regular validation against principles during development and before releases.

**Version**: 2.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-09