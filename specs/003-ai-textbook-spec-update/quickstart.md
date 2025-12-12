# Quickstart Guide: AI-Native Textbook on Physical AI & Humanoid Robotics

## Prerequisites

- Ubuntu 22.04 LTS
- Python 3.11+
- Node.js 18+ and npm
- Docker and Docker Compose
- ROS 2 Humble Hawksbill
- Git

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-textbook-physical-ai-humanoid-robotics
```

### 2. Backend Setup (FastAPI)
```bash
cd backend
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration (database URLs, API keys, etc.)
```

### 3. Frontend Setup (Docusaurus)
```bash
cd frontend
npm install
```

### 4. Database Setup
```bash
# Set up Neon Postgres database
# The application will handle schema migrations on first run

# Set up Qdrant vector database
docker-compose up -d qdrant
```

### 5. Simulation Environment Setup
Choose your preferred simulation environment:

#### Gazebo Setup
```bash
# Install ROS 2 Humble with Gazebo Garden
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

#### Isaac Sim Setup
```bash
# Download and install Isaac Sim from NVIDIA
# Follow the official Isaac Sim installation guide
```

### 6. Environment Variables
Create `.env` files in both backend and frontend directories:

**Backend `.env`:**
```
DATABASE_URL=postgresql://user:password@localhost:5432/textbook_db
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key
OPENAI_API_KEY=your-openai-api-key
CHATKIT_API_KEY=your-chatkit-api-key
SECRET_KEY=your-secret-key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

**Frontend `.env`:**
```
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_CHATKIT_API_KEY=your-chatkit-api-key
```

## Running the Application

### 1. Start Backend Services
```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Start Frontend
```bash
cd frontend
npm start
```

### 3. Start Simulation Environment (Optional)
```bash
# For Gazebo:
ros2 launch gazebo_ros empty_world.launch.py

# For Isaac Sim:
# Launch Isaac Sim from your installation
```

## API Endpoints

### Textbook Content
- `GET /api/v1/textbook/chapters` - List all chapters
- `GET /api/v1/textbook/chapters/{slug}` - Get specific chapter
- `GET /api/v1/textbook/tutorials` - List all tutorials

### AI Tutor (RAG)
- `POST /api/v1/ai-tutor/chat` - Chat with AI tutor
- `POST /api/v1/ai-tutor/query` - Ask questions about textbook content

### Personalization
- `GET /api/v1/personalization/settings` - Get user preferences
- `PUT /api/v1/personalization/settings` - Update user preferences

### Simulation
- `POST /api/v1/simulation/run` - Start a simulation session
- `GET /api/v1/simulation/{session_id}` - Get simulation status
- `DELETE /api/v1/simulation/{session_id}` - Stop simulation

## Development Workflow

### Adding a New Chapter
1. Create a new MDX file in `frontend/content/chapters/`
2. Ensure word count is between 1,500-2,500 words
3. Add learning outcomes at the beginning
4. Include relevant code examples and diagrams
5. Update the backend data model if needed
6. Re-index content for RAG: `python -m src.utils.vectorizer --rebuild`

### Running Tests
```bash
# Backend tests
cd backend
python -m pytest tests/

# Frontend tests
cd frontend
npm test
```

## Troubleshooting

### Common Issues
- **API requests timing out**: Ensure backend is running on port 8000
- **RAG queries failing**: Check that Qdrant is running and populated with vectors
- **Simulation not connecting**: Verify ROS 2 installation and network configuration
- **Translation not working**: Ensure language models are properly configured

### Performance Optimization
- For RAG response time: Optimize vector database indexing and implement caching
- For simulation performance: Ensure adequate GPU resources for Jetson/RTX targets
- For content loading: Implement proper asset optimization and CDN configuration