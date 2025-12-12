# AI-Native Textbook on Physical AI & Humanoid Robotics

An interactive, AI-powered textbook for learning Physical AI and Humanoid Robotics with integrated simulation, personalization, and intelligent tutoring.

## Features

- **Interactive AI Tutor**: Chat-based AI assistant that explains robotics concepts and helps with code
- **Personalization**: Content adapts to your learning level and preferences
- **Urdu Translation**: Technical content available in Urdu
- **Integrated Simulations**: Run Gazebo, Isaac Sim, and Unity simulations directly from the textbook
- **ROS 2 Integration**: Complete tutorials and examples using ROS 2 Humble
- **VLA Pipeline**: Vision-Language-Action integration for speech-to-robot-control

## Architecture

### Backend Services
- **FastAPI**: Main backend API with RAG-based AI tutor
- **Qdrant**: Vector database for content retrieval
- **PostgreSQL**: User data and preferences storage
- **OpenAI API**: AI tutoring and translation services

### Frontend
- **Docusaurus**: Static site generation with MDX support
- **React Components**: Interactive tutoring and personalization controls
- **API Client**: Integration with backend services

### Simulation Integration
- **Gazebo**: Physics simulation with ROS 2 integration
- **Isaac Sim**: NVIDIA's robotics simulation platform
- **Unity**: ML-Agents for reinforcement learning scenarios

## Getting Started

### Prerequisites
- Ubuntu 22.04 LTS
- Python 3.11+
- Node.js 18+
- Docker and Docker Compose
- ROS 2 Humble Hawksbill (for simulation)

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd ai-textbook-physical-ai-humanoid-robotics
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

3. Install backend dependencies:
```bash
cd backend
pip install -r requirements.txt
```

4. Install frontend dependencies:
```bash
cd ../my-website
yarn install
```

### Running the Application

#### Development Mode
```bash
# Terminal 1: Start backend
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Terminal 2: Start frontend
cd my-website
yarn start
```

#### Production Mode (Docker)
```bash
docker-compose up --build
```

## API Endpoints

### AI Tutor
- `POST /api/v1/ai-tutor/chat` - Chat with the AI tutor
- `POST /api/v1/ai-tutor/query` - Query textbook content

### Simulation
- `POST /api/v1/simulation/start` - Start a simulation
- `POST /api/v1/simulation/stop` - Stop a simulation
- `GET /api/v1/simulation/status/{id}` - Get simulation status

### Personalization
- `GET /api/v1/users/{id}/preferences` - Get user preferences
- `PUT /api/v1/users/{id}/preferences` - Update user preferences

### Content
- `POST /api/v1/translate` - Translate content
- `POST /api/v1/search` - Search textbook content

## ROS 2 Integration

The textbook includes complete ROS 2 tutorials with:

- Basic publisher/subscriber examples
- Service and action implementations
- Robot control with ros2_control
- Navigation and manipulation examples
- URDF models for humanoid robots

### Running ROS 2 Examples
```bash
cd simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch tutorials pub_sub_launch.py
```

## Simulation Environments

### Gazebo
- Humanoid robot models with physics
- Environment scenarios
- Sensor simulation

### Isaac Sim
- NVIDIA RTX-accelerated physics
- Advanced sensor simulation
- GPU-accelerated training environments

### Unity
- ML-Agents reinforcement learning
- Custom physics environments
- VR/AR support

## Content Structure

The textbook follows this structure:

1. Introduction to Physical AI
2. ROS 2 Fundamentals
3. Robotics Fundamentals
4. Humanoid Control Systems
5. Simulation Integration
6. Vision-Language-Action Pipelines
7. Advanced Robotics Concepts

Each chapter includes:
- Overview and learning objectives
- Theory and concepts
- Code examples and tutorials
- Simulation integration
- Interactive quizzes

## Personalization Features

The system adapts to user preferences:

- **Learning Level**: Beginner, Intermediate, Advanced
- **Learning Style**: Visual, Auditory, Kinesthetic, Reading/Writing
- **Interests**: Customize content based on user interests
- **Accessibility**: Support for different accessibility needs

## Translation Support

Content is available in multiple languages:

- English (primary)
- Urdu (with technical accuracy preserved)
- Additional languages can be added

## Development

### Adding New Chapters

1. Create a new MDX file in `my-website/content/chapters/`
2. Add the chapter to `my-website/sidebars.js`
3. Include personalization and translation hooks

### Adding New Simulations

1. Create ROS 2 packages in `simulation/ros2_ws/src/`
2. Add Gazebo models to `simulation/gazebo/models/`
3. Create Isaac Sim configurations in `simulation/isaac_sim/`
4. Add Unity assets to `simulation/unity/Assets/`

### Testing

Run backend tests:
```bash
cd backend
pytest tests/
```

## Deployment

### Production Deployment

For production deployment, use the Docker Compose configuration:

```bash
# Build and start all services
docker-compose up --build -d

# Monitor services
docker-compose logs -f
```

### Environment Variables

Required environment variables:

- `DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant vector database URL
- `OPENAI_API_KEY`: OpenAI API key for AI services
- `SECRET_KEY`: Secret key for authentication
- `API_BASE_URL`: Base URL for frontend API calls

## Performance Optimization

The system is optimized for:

- **RAG Response Time**: < 2-3 seconds for 95% of queries
- **Resource Usage**: Optimized for Jetson and RTX GPU platforms
- **Scalability**: Docker-based deployment for easy scaling

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support, please open an issue in the GitHub repository or contact the development team.