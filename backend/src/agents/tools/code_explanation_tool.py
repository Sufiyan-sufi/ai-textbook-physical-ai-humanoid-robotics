from typing import Dict, Any
import ast
import re


class CodeExplanationTool:
    """Tool for explaining ROS 2 code concepts and providing debugging help"""

    def __init__(self):
        self.ros2_patterns = {
            'publisher': r'create_publisher\([^)]+\)',
            'subscriber': r'create_subscription\([^)]+\)',
            'service_server': r'create_service\([^)]+\)',
            'service_client': r'create_client\([^)]+\)',
            'action_server': r'create_action_server\([^)]+\)',
            'action_client': r'create_action_client\([^)]+\)',
            'timer': r'create_timer\([^)]+\)',
            'parameter': r'declare_parameter\([^)]+\)',
        }

    def explain_code(self, code_snippet: str, language: str = "python", context: str = "") -> Dict[str, Any]:
        """
        Explain ROS 2 code concepts and provide debugging help
        """
        try:
            explanation = {
                "status": "success",
                "language": language,
                "concepts_identified": [],
                "explanation": "",
                "best_practices": [],
                "potential_issues": [],
                "suggestions": []
            }

            # Analyze the code based on language
            if language.lower() == "python":
                explanation.update(self._analyze_python_ros2_code(code_snippet))
            elif language.lower() == "cpp":
                explanation.update(self._analyze_cpp_ros2_code(code_snippet))
            else:
                explanation["explanation"] = f"Code analysis for {language} is not fully supported yet. Here's a general explanation."

            # Add context-specific explanation
            if context:
                explanation["context_explanation"] = self._add_context_explanation(context, code_snippet)

            # Add common ROS 2 patterns
            patterns_found = self._find_ros2_patterns(code_snippet)
            explanation["concepts_identified"].extend(patterns_found)

            return explanation

        except Exception as e:
            return {
                "status": "error",
                "message": f"Error explaining code: {str(e)}",
                "explanation": "Unable to analyze the code snippet. Please check the code syntax and try again."
            }

    def _analyze_python_ros2_code(self, code: str) -> Dict[str, Any]:
        """Analyze Python ROS 2 code"""
        result = {
            "concepts_identified": [],
            "explanation": "",
            "best_practices": [],
            "potential_issues": [],
            "suggestions": []
        }

        # Look for common ROS 2 Python patterns
        if 'rclpy.init' in code:
            result["concepts_identified"].append("rclpy initialization")
            result["explanation"] += "This code initializes the ROS 2 client library for Python (rclpy). "
            result["best_practices"].append("Always call rclpy.init() before creating nodes")

        if 'Node' in code and 'rclpy.node' in code:
            result["concepts_identified"].append("ROS 2 Node")
            result["explanation"] += "This defines a ROS 2 Node, which is the basic execution unit in ROS 2. "
            result["best_practices"].append("Nodes should have descriptive names and proper lifecycle management")

        # Check for publishers
        if 'create_publisher' in code:
            result["concepts_identified"].append("Publisher")
            result["explanation"] += "This creates a publisher to send messages on a topic. "
            result["best_practices"].append("Use appropriate QoS settings for your use case")

        # Check for subscribers
        if 'create_subscription' in code:
            result["concepts_identified"].append("Subscriber")
            result["explanation"] += "This creates a subscriber to receive messages from a topic. "
            result["best_practices"].append("Ensure callback functions are efficient and don't block")

        # Check for services
        if 'create_service' in code or 'create_client' in code:
            result["concepts_identified"].append("Service/Client")
            result["explanation"] += "This implements a service-server pattern for request-response communication. "
            result["best_practices"].append("Handle service calls asynchronously when possible")

        # Check for actions
        if 'action' in code.lower() and ('server' in code.lower() or 'client' in code.lower()):
            result["concepts_identified"].append("Action")
            result["explanation"] += "This implements an action for long-running tasks with feedback. "
            result["best_practices"].append("Use actions for tasks that take time and need feedback/progress updates")

        # Check for parameter declarations
        if 'declare_parameter' in code:
            result["concepts_identified"].append("Parameter")
            result["explanation"] += "This declares a parameter that can be configured externally. "
            result["best_practices"].append("Provide default values for all parameters")

        return result

    def _analyze_cpp_ros2_code(self, code: str) -> Dict[str, Any]:
        """Analyze C++ ROS 2 code"""
        result = {
            "concepts_identified": [],
            "explanation": "",
            "best_practices": [],
            "potential_issues": [],
            "suggestions": []
        }

        # Look for common ROS 2 C++ patterns
        if '#include "rclcpp/rclcpp.hpp"' in code:
            result["concepts_identified"].append("rclcpp header")
            result["explanation"] += "This includes the main ROS 2 C++ client library header. "
            result["best_practices"].append("Include rclcpp.hpp for ROS 2 C++ functionality")

        if 'rclcpp::Node' in code:
            result["concepts_identified"].append("ROS 2 Node")
            result["explanation"] += "This defines a ROS 2 Node in C++, which is the basic execution unit. "
            result["best_practices"].append("Nodes should have proper constructors and inheritance")

        if 'create_publisher' in code:
            result["concepts_identified"].append("Publisher")
            result["explanation"] += "This creates a publisher to send messages on a topic. "
            result["best_practices"].append("Use std::make_shared for message creation")

        if 'create_subscription' in code:
            result["concepts_identified"].append("Subscriber")
            result["explanation"] += "This creates a subscriber to receive messages from a topic. "
            result["best_practices"].append("Use lambda functions or member functions for callbacks")

        return result

    def _add_context_explanation(self, context: str, code: str) -> str:
        """Add explanation based on the provided context"""
        if "navigation" in context.lower():
            return "This code appears to be related to navigation. Consider using the Navigation2 stack for advanced navigation capabilities."
        elif "manipulation" in context.lower():
            return "This code appears to be related to manipulation. Consider using MoveIt2 for motion planning and manipulation."
        elif "perception" in context.lower():
            return "This code appears to be related to perception. Consider using perception packages like image_transport and cv_bridge for image processing."
        elif "control" in context.lower():
            return "This code appears to be related to control. Consider using ros2_control for hardware abstraction and control."
        else:
            return f"Context: {context}. This code is related to the specified area of robotics."

    def _find_ros2_patterns(self, code: str) -> list:
        """Find common ROS 2 patterns in the code"""
        patterns_found = []
        for pattern_name, pattern in self.ros2_patterns.items():
            if re.search(pattern, code):
                patterns_found.append(pattern_name)
        return patterns_found


# Example usage and testing
if __name__ == "__main__":
    tool = CodeExplanationTool()

    # Test Python ROS 2 code
    python_code = """
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
"""

    result = tool.explain_code(python_code, "python", "basic publisher tutorial")
    print("Python Code Explanation:")
    print(f"Concepts: {result['concepts_identified']}")
    print(f"Explanation: {result['explanation']}")
    print(f"Best Practices: {result['best_practices']}")