# Data Model: Textbook on Physical AI & Humanoid Robotics

## Entities

### Chapter
- **Fields**: id, title, content, wordCount, learningOutcomes, summary, quizQuestions, diagrams, glossary, prerequisites
- **Relationships**: Contains multiple Tutorials and Examples
- **Validation**: Must have title, content, and meet word count requirements (4,500-6,000 words)

### Tutorial
- **Fields**: id, title, description, steps, codeExamples, simulationRequirements, learningObjectives, prerequisites
- **Relationships**: Belongs to a Chapter; contains multiple Code Examples
- **Validation**: Must be reproducible in simulation environments, include clear steps and learning objectives

### Example
- **Fields**: id, title, description, code, simulationEnvironment, expectedOutput, explanation
- **Relationships**: Belongs to a Chapter
- **Validation**: Must be executable and free from hallucinated APIs or commands

### Source
- **Fields**: id, title, authors, publication, year, type, url, verificationStatus
- **Relationships**: Referenced by Chapters and Tutorials
- **Validation**: Must be from credible sources (peer-reviewed papers, technical documentation)

### Humanoid Model
- **Fields**: id, name, description, simulationEnvironment, joints, sensors, capabilities
- **Relationships**: Used in Tutorials and Examples
- **Validation**: Must be compatible with specified simulation environment

### Simulation Environment
- **Fields**: id, name, type (Gazebo, Unity, IsaacSim), version, requirements, capabilities
- **Relationships**: Contains multiple Humanoid Models
- **Validation**: Must be accessible and reproducible for educational use