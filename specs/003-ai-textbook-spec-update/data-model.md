# Data Model: AI-Native Textbook on Physical AI & Humanoid Robotics

## Core Entities

### Chapter
- **Fields**:
  - id: UUID (primary key)
  - title: string (chapter title)
  - slug: string (URL-friendly identifier)
  - content: string (MDX content)
  - word_count: integer (1,500-2,500 as required)
  - learning_outcomes: array of strings (specific learning objectives)
  - prerequisites: array of strings (required knowledge)
  - next_chapters: array of UUIDs (navigation links)
  - created_at: datetime
  - updated_at: datetime
  - version: integer (for content versioning)
- **Validation**:
  - word_count must be between 1,500 and 2,500
  - title and content are required
  - slug must be unique
- **Relationships**:
  - One-to-many with Tutorial
  - One-to-many with LearningOutcome

### Tutorial
- **Fields**:
  - id: UUID (primary key)
  - chapter_id: UUID (foreign key to Chapter)
  - title: string (tutorial title)
  - content: string (step-by-step instructions)
  - simulation_environment: enum ('gazebo', 'isaac', 'unity')
  - hardware_target: enum ('jetson', 'rtx', 'simulation-only')
  - estimated_duration: integer (in minutes)
  - code_examples: array of strings (code snippets)
  - learning_outcomes: array of strings
  - created_at: datetime
  - updated_at: datetime
- **Validation**:
  - chapter_id must reference an existing chapter
  - content is required
  - simulation_environment must be one of allowed values
- **Relationships**:
  - Many-to-one with Chapter

### User
- **Fields**:
  - id: UUID (primary key)
  - email: string (unique, required)
  - name: string (user display name)
  - learning_preferences: JSON (personalization settings)
  - language_preference: string (default 'en', 'ur' for Urdu)
  - progress_tracking: JSON (chapter completion status)
  - created_at: datetime
  - updated_at: datetime
- **Validation**:
  - email must be unique and valid
  - name is optional but recommended
- **Relationships**:
  - One-to-many with UserProgress
  - One-to-many with PersonalizationSetting

### UserProgress
- **Fields**:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User)
  - chapter_id: UUID (foreign key to Chapter)
  - completion_percentage: float (0.0 to 1.0)
  - time_spent_seconds: integer
  - last_accessed: datetime
  - quiz_scores: array of JSON (quiz results)
  - created_at: datetime
  - updated_at: datetime
- **Validation**:
  - user_id and chapter_id must reference existing records
  - completion_percentage must be between 0.0 and 1.0
- **Relationships**:
  - Many-to-one with User
  - Many-to-one with Chapter

### SimulationSession
- **Fields**:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User)
  - chapter_id: UUID (foreign key to Chapter)
  - tutorial_id: UUID (foreign key to Tutorial)
  - environment: enum ('gazebo', 'isaac', 'unity')
  - status: enum ('running', 'completed', 'failed', 'terminated')
  - parameters: JSON (simulation configuration)
  - created_at: datetime
  - started_at: datetime
  - completed_at: datetime
- **Validation**:
  - All foreign keys must reference existing records
  - status must be one of allowed values
- **Relationships**:
  - Many-to-one with User
  - Many-to-one with Chapter
  - Many-to-one with Tutorial

### AIChatSession
- **Fields**:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User)
  - chapter_id: UUID (foreign key to Chapter, optional)
  - session_data: JSON (chat history and context)
  - created_at: datetime
  - updated_at: datetime
- **Validation**:
  - user_id must reference an existing user
- **Relationships**:
  - Many-to-one with User
  - Many-to-one with Chapter (optional)

### ContentVector
- **Fields**:
  - id: UUID (primary key)
  - chapter_id: UUID (foreign key to Chapter)
  - content_section: string (the text being vectorized)
  - vector_embedding: array of floats (Qdrant vector)
  - metadata: JSON (additional context like section type)
  - created_at: datetime
- **Validation**:
  - chapter_id must reference an existing chapter
  - vector_embedding must be properly formatted for Qdrant
- **Relationships**:
  - Many-to-one with Chapter

## State Transitions

### SimulationSession States
- `created` → `running` (when simulation starts)
- `running` → `completed` (when simulation completes successfully)
- `running` → `failed` (when simulation encounters an error)
- `running` → `terminated` (when user stops the simulation)

### UserProgress States
- Progress percentage increases as user interacts with content
- Quiz scores are added as user completes assessments
- Last accessed timestamp updates with each content interaction

## Indexes
- User.email (unique)
- Chapter.slug (unique)
- ContentVector.chapter_id (for fast lookup during RAG)
- UserProgress.user_id and chapter_id (composite for progress tracking)