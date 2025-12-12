# Quickstart Guide: Textbook on Physical AI & Humanoid Robotics

## Prerequisites

1. **System Requirements**:
   - Linux (Ubuntu 22.04 LTS recommended) or WSL2 on Windows
   - 8GB+ RAM, 50GB+ free disk space
   - Python 3.11+
   - Node.js 18+
   - Docker (for containerized simulations)

2. **Software Dependencies**:
   ```bash
   # ROS 2 Humble Hawksbill
   sudo apt update && sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -gpg
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash

   # Node.js and npm
   curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
   sudo apt install -y nodejs

   # Python virtual environment
   python3 -m venv venv
   source venv/bin/activate
   pip install --upgrade pip
   ```

## Setup Instructions

1. **Clone and Initialize the Repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   source venv/bin/activate  # If using virtual environment
   ```

2. **Install Backend Dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Install Frontend Dependencies**:
   ```bash
   cd ../my-website
   npm install
   ```

4. **Setup Simulation Environments** (Optional for Full Experience):
   ```bash
   # For Gazebo
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

   # For Unity ML-Agents (requires Unity Hub installation)
   # Download Unity Hub from unity3d.com and install Unity 2022.3 LTS

   # For NVIDIA Isaac Sim (requires NVIDIA account)
   # Follow instructions at developer.nvidia.com/isaac-sim
   ```

## Running the Application

1. **Start the Backend Server**:
   ```bash
   cd backend
   source venv/bin/activate
   uvicorn src.api.main:app --reload --port 8000
   ```

2. **Start the Docusaurus Frontend**:
   ```bash
   cd my-website
   npm start
   ```
   The textbook will be available at http://localhost:3000

3. **Running Simulations**:
   - Tutorials with simulation requirements will provide specific instructions
   - Most simulations can be run through the interactive elements in the textbook
   - Some advanced examples may require launching ROS 2 nodes separately

## Development Workflow

1. **Adding Content**:
   - New chapters go in `my-website/docs/chapters/`
   - Tutorials go in `my-website/docs/tutorials/`
   - Update `my-website/sidebars.js` to add new content to navigation

2. **Backend API Development**:
   - API endpoints defined in `backend/src/api/`
   - Models defined in `backend/src/models/`
   - Services defined in `backend/src/services/`

3. **Testing**:
   ```bash
   # Backend tests
   cd backend
   pytest

   # Frontend tests
   cd my-website
   npm test
   ```

## Troubleshooting

1. **Simulation Issues**:
   - Ensure proper GPU drivers are installed for graphics-intensive simulations
   - Check ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`

2. **Frontend Build Issues**:
   - Clear npm cache: `npm start -- --clear-cache`
   - Delete node_modules and reinstall if needed

3. **Backend Connection Issues**:
   - Verify backend is running on port 8000
   - Check CORS settings in backend configuration