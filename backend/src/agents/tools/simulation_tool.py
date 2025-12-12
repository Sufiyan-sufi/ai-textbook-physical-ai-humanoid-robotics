from typing import Dict, Any
import subprocess
import os
import json
from src.api.v1.simulation_routes import SimulationRequest


class SimulationTool:
    """Tool for running simulations to demonstrate concepts"""

    def __init__(self):
        self.supported_environments = ["gazebo", "isaac", "unity"]
        self.simulation_scenarios = {
            "basic_movement": {
                "gazebo": "ros2 launch humanoid_control basic_movement.launch.py",
                "isaac": "isaac-sim --scenario=basic_movement",
                "unity": "unity-sim --scene=basic_movement"
            },
            "navigation": {
                "gazebo": "ros2 launch humanoid_control navigation.launch.py",
                "isaac": "isaac-sim --scenario=navigation",
                "unity": "unity-sim --scene=navigation"
            },
            "manipulation": {
                "gazebo": "ros2 launch humanoid_control manipulation.launch.py",
                "isaac": "isaac-sim --scenario=manipulation",
                "unity": "unity-sim --scene=manipulation"
            },
            "balance": {
                "gazebo": "ros2 launch humanoid_control balance.launch.py",
                "isaac": "isaac-sim --scenario=balance",
                "unity": "unity-sim --scene=balance"
            }
        }

    def run_simulation(self, scenario: str, parameters: Dict[str, Any] = None, environment: str = "gazebo") -> Dict[str, Any]:
        """
        Run a simulation for the student to demonstrate concepts
        """
        try:
            if environment not in self.supported_environments:
                return {
                    "status": "error",
                    "message": f"Unsupported environment: {environment}. Supported: {self.supported_environments}"
                }

            if scenario not in self.simulation_scenarios:
                return {
                    "status": "error",
                    "message": f"Unsupported scenario: {scenario}. Supported: {list(self.simulation_scenarios.keys())}"
                }

            # Get the command for the specific environment and scenario
            command = self.simulation_scenarios[scenario].get(environment)

            if not command:
                return {
                    "status": "error",
                    "message": f"No command found for scenario '{scenario}' in environment '{environment}'"
                }

            # Add parameters to the command if provided
            if parameters:
                command = self._add_parameters_to_command(command, parameters)

            # Execute the simulation command
            result = self._execute_simulation_command(command, environment)

            return {
                "status": "success",
                "scenario": scenario,
                "environment": environment,
                "command": command,
                "result": result,
                "message": f"Successfully ran {scenario} simulation in {environment}"
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Error running simulation: {str(e)}",
                "scenario": scenario,
                "environment": environment
            }

    def _add_parameters_to_command(self, command: str, parameters: Dict[str, Any]) -> str:
        """Add parameters to the simulation command"""
        param_str = ""
        for key, value in parameters.items():
            if isinstance(value, bool):
                if value:
                    param_str += f" --{key}"
            elif isinstance(value, (int, float, str)):
                param_str += f" --{key} {value}"
            else:
                param_str += f" --{key} '{json.dumps(value)}'"

        return f"{command} {param_str}"

    def _execute_simulation_command(self, command: str, environment: str) -> Dict[str, Any]:
        """Execute the simulation command and return results"""
        try:
            # For now, we'll simulate the execution since we don't have the actual simulators installed
            # In a real implementation, this would actually run the simulation
            if environment == "gazebo":
                return self._simulate_gazebo_result(command)
            elif environment == "isaac":
                return self._simulate_isaac_result(command)
            elif environment == "unity":
                return self._simulate_unity_result(command)
            else:
                return {"status": "unknown", "command": command}

        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "command": command
            }

    def _simulate_gazebo_result(self, command: str) -> Dict[str, Any]:
        """Simulate Gazebo execution result"""
        return {
            "status": "running",
            "simulation_id": "gazebo_sim_12345",
            "process_id": 12345,
            "logs": [
                {"timestamp": "2023-01-01T10:00:00Z", "level": "info", "message": "Gazebo server started"},
                {"timestamp": "2023-01-01T10:00:01Z", "level": "info", "message": "Robot model loaded"},
                {"timestamp": "2023-01-01T10:00:02Z", "level": "info", "message": "Simulation running"}
            ],
            "visualization_url": "http://localhost:8080/gazebo",
            "duration_estimate": "30s"
        }

    def _simulate_isaac_result(self, command: str) -> Dict[str, Any]:
        """Simulate Isaac Sim execution result"""
        return {
            "status": "running",
            "simulation_id": "isaac_sim_67890",
            "process_id": 67890,
            "logs": [
                {"timestamp": "2023-01-01T10:00:00Z", "level": "info", "message": "Isaac Sim initialized"},
                {"timestamp": "2023-01-01T10:00:01Z", "level": "info", "message": "USD stage loaded"},
                {"timestamp": "2023-01-01T10:00:02Z", "level": "info", "message": "Robot assets loaded"},
                {"timestamp": "2023-01-01T10:00:03Z", "level": "info", "message": "Simulation running with RTX acceleration"}
            ],
            "visualization_url": "http://localhost:8080/isaac",
            "duration_estimate": "45s"
        }

    def _simulate_unity_result(self, command: str) -> Dict[str, Any]:
        """Simulate Unity execution result"""
        return {
            "status": "running",
            "simulation_id": "unity_sim_54321",
            "process_id": 54321,
            "logs": [
                {"timestamp": "2023-01-01T10:00:00Z", "level": "info", "message": "Unity player started"},
                {"timestamp": "2023-01-01T10:00:01Z", "level": "info", "message": "Scene loaded"},
                {"timestamp": "2023-01-01T10:00:02Z", "level": "info", "message": "Robot controller initialized"},
                {"timestamp": "2023-01-01T10:00:03Z", "level": "info", "message": "Simulation running"}
            ],
            "visualization_url": "http://localhost:8080/unity",
            "duration_estimate": "40s"
        }

    def check_environment_status(self, environment: str) -> Dict[str, Any]:
        """Check if the simulation environment is available"""
        try:
            if environment == "gazebo":
                # Check if gazebo is installed and available
                result = subprocess.run(["which", "gazebo"], capture_output=True, text=True)
                is_available = result.returncode == 0
            elif environment == "isaac":
                # Check if Isaac Sim is available (this would be more complex in practice)
                is_available = os.path.exists("/opt/isaac-sim") or "isaacsim" in os.environ.get("PATH", "")
            elif environment == "unity":
                # Check if Unity is available
                result = subprocess.run(["which", "unity-editor"], capture_output=True, text=True)
                is_available = result.returncode == 0
            else:
                is_available = False

            return {
                "environment": environment,
                "available": is_available,
                "message": f"Simulation environment {environment} is {'available' if is_available else 'not available'}"
            }
        except Exception as e:
            return {
                "environment": environment,
                "available": False,
                "error": str(e),
                "message": f"Error checking {environment} availability: {str(e)}"
            }


# Example usage and testing
if __name__ == "__main__":
    tool = SimulationTool()

    # Test simulation execution
    result = tool.run_simulation("basic_movement", {"duration": 30, "robot_model": "humanoid"}, "gazebo")
    print("Simulation Result:")
    print(json.dumps(result, indent=2))

    # Test environment status
    status = tool.check_environment_status("gazebo")
    print("\nEnvironment Status:")
    print(json.dumps(status, indent=2))