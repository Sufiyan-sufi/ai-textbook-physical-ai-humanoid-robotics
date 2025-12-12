# Unity Simulation for AI-Native Textbook

This Unity project provides simulation capabilities for the humanoid robot described in the AI-Native Textbook on Physical AI & Humanoid Robotics.

## Project Structure

- `Assets/Scenes/` - Unity scenes for different simulation environments
- `Assets/Models/` - 3D models for the humanoid robot and environment
- `Assets/Scripts/` - C# scripts for robot control and ROS integration
- `Assets/Materials/` - Materials and textures for the robot and environment

## Components

### RobotController.cs
The main script that handles robot control, including:
- Joint position control
- Basic movement commands
- Placeholder for ROS communication

## ROS Integration

This project is designed to work with ROS through the ROS# library or similar Unity-ROS bridges. The RobotController.cs script includes placeholder methods for receiving ROS commands and publishing sensor data.

## Setup Instructions

1. Open the project in Unity 2022.3 or later
2. Import the ROS# package for ROS communication (if using ROS integration)
3. Configure the robot joints in the RobotController component
4. Build and run the simulation

## Simulation Environments

Different scenes are provided for various simulation scenarios:
- Basic environment for testing joint movements
- Navigation environment with obstacles
- Manipulation environment with objects to interact with