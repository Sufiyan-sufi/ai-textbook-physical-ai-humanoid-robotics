from typing import Dict, Any
from src.services.rag_service import RAGService


class ConceptExplanationTool:
    """Tool for explaining robotics concepts from the textbook content"""

    def __init__(self):
        self.rag_service = RAGService()
        self.concept_categories = {
            "physical_ai": [
                "embodied cognition", "sensorimotor learning", "morphological computation",
                "affordance", "active inference", "hierarchical control"
            ],
            "ros2": [
                "nodes", "topics", "services", "actions", "parameters",
                "lifecycle", "composition", "quality of service"
            ],
            "humanoid_control": [
                "inverse kinematics", "center of mass control", "zero moment point",
                "walking patterns", "balance control", "motion planning"
            ],
            "simulation": [
                "physics engines", "sensor simulation", "real-time constraints",
                "gazebo", "isaac sim", "unity ml-agents"
            ],
            "vla": [
                "vision-language-action models", "multimodal perception",
                "grounding", "planning", "execution"
            ]
        }

    def explain_concept(self, concept: str, level: str = "beginner", examples: bool = True) -> Dict[str, Any]:
        """
        Explain robotics concepts from the textbook content
        """
        try:
            explanation = {
                "status": "success",
                "concept": concept,
                "level": level,
                "definition": "",
                "detailed_explanation": "",
                "examples": [],
                "related_concepts": [],
                "mathematical_formulation": "",
                "practical_applications": [],
                "textbook_references": []
            }

            # Find the concept in our categories
            category = self._categorize_concept(concept)
            explanation["category"] = category

            # Generate explanation based on level
            explanation.update(self._generate_explanation(concept, level, examples))

            # Search for related content in the textbook
            related_content = self.rag_service.query(concept, f"explanation for {concept} at {level} level")
            explanation["textbook_references"] = related_content

            return explanation

        except Exception as e:
            return {
                "status": "error",
                "message": f"Error explaining concept: {str(e)}",
                "explanation": "Unable to explain the concept. Please try a different concept or check the spelling."
            }

    def _categorize_concept(self, concept: str) -> str:
        """Categorize a concept into one of our predefined categories"""
        concept_lower = concept.lower()

        for category, concepts in self.concept_categories.items():
            for cat_concept in concepts:
                if cat_concept in concept_lower:
                    return category

        # If not found in specific categories, return general
        return "general"

    def _generate_explanation(self, concept: str, level: str, examples: bool) -> Dict[str, Any]:
        """Generate explanation based on concept and level"""
        result = {
            "definition": self._get_definition(concept, level),
            "detailed_explanation": self._get_detailed_explanation(concept, level),
            "related_concepts": self._get_related_concepts(concept),
            "mathematical_formulation": self._get_mathematical_formulation(concept),
            "practical_applications": self._get_practical_applications(concept)
        }

        if examples:
            result["examples"] = self._get_examples(concept, level)

        return result

    def _get_definition(self, concept: str, level: str) -> str:
        """Get a definition appropriate for the level"""
        concept_lower = concept.lower()

        # Common robotics concepts with definitions
        definitions = {
            "embodied cognition": {
                "beginner": "The idea that intelligence emerges from the interaction between an agent and its environment, rather than existing purely in the brain or computer.",
                "intermediate": "A theory in cognitive science and robotics that intelligence is shaped by the body's interactions with the environment, emphasizing the role of physical form in cognition.",
                "advanced": "A framework where cognitive processes are understood as arising from the dynamic coupling between the nervous system, body, and environment, with implications for robot design and AI."
            },
            "ros2": {
                "beginner": "Robot Operating System 2, a flexible framework for writing robot software that provides services like hardware abstraction, device drivers, and message passing.",
                "intermediate": "A middleware framework for robotics applications that provides distributed computing capabilities, package management, and communication between different software components.",
                "advanced": "A collection of libraries and tools that provide low-level functionality for robotic applications, including DDS-based communication, lifecycle management, and real-time capabilities."
            },
            "inverse kinematics": {
                "beginner": "The process of calculating the joint angles needed to position a robot's end effector at a desired location.",
                "intermediate": "A mathematical approach to determine the configuration of a robot manipulator that achieves a specified end-effector pose.",
                "advanced": "A nonlinear optimization problem that maps Cartesian space coordinates to joint space coordinates, often solved using Jacobian-based or geometric methods."
            },
            "zero moment point": {
                "beginner": "A point where the net moment of the ground reaction force is zero, used to determine the stability of walking robots.",
                "intermediate": "A stability criterion in humanoid robotics that indicates where the ground reaction force would need to act to produce no moment about that point.",
                "advanced": "A mathematical concept in dynamics used to analyze the balance of bipedal robots, calculated from the center of mass position and acceleration."
            }
        }

        # Return specific definition if available, otherwise generic
        if concept_lower in definitions:
            level_defs = definitions[concept_lower]
            return level_defs.get(level, level_defs.get("beginner", f"A concept in robotics related to {concept}"))
        else:
            return f"A concept in robotics related to {concept}. This concept is important for understanding physical AI and humanoid robotics."

    def _get_detailed_explanation(self, concept: str, level: str) -> str:
        """Get a detailed explanation appropriate for the level"""
        concept_lower = concept.lower()

        # Detailed explanations for common concepts
        explanations = {
            "embodied cognition": {
                "beginner": "In robotics, embodied cognition means that a robot's intelligence comes from its interaction with the physical world. Instead of just processing information in a computer, the robot learns and thinks through its body and sensors. This approach helps robots better understand and navigate the real world.",
                "intermediate": "Embodied cognition in robotics emphasizes that cognitive processes are deeply influenced by the physical body and its interactions with the environment. This perspective has led to more robust and adaptive robotic systems that can handle real-world complexity better than purely symbolic AI approaches.",
                "advanced": "The embodied cognition framework in robotics challenges traditional computational approaches by arguing that cognition emerges from the dynamic interaction between an agent's morphology, sensors, actuators, and environment. This has implications for robot design, learning algorithms, and the development of truly intelligent systems."
            },
            "ros2": {
                "beginner": "ROS2 is a collection of software libraries and tools that help you build robot applications. It handles communication between different parts of your robot, provides common tools for debugging, and helps you reuse code from other robot projects.",
                "intermediate": "ROS2 provides a middleware layer that enables distributed computing for robotics applications. It uses DDS (Data Distribution Service) for communication, provides package management, and includes tools for simulation, visualization, and debugging of robotic systems.",
                "advanced": "ROS2 implements a modern, secure, and real-time capable middleware for robotics based on DDS standards. It provides lifecycle management, composition capabilities, and real-time performance features essential for production robotic systems."
            }
        }

        if concept_lower in explanations:
            level_expls = explanations[concept_lower]
            return level_expls.get(level, level_expls.get("beginner", f"Provides detailed information about {concept}"))
        else:
            return f"Provides detailed information about {concept} in the context of robotics and physical AI."

    def _get_examples(self, concept: str, level: str) -> list:
        """Get examples appropriate for the level"""
        concept_lower = concept.lower()

        examples = {
            "embodied cognition": [
                {
                    "beginner": "A robot that learns to walk by actually walking, rather than just calculating walking motions in a computer",
                    "intermediate": "The iCub humanoid robot learning object manipulation through physical interaction with objects",
                    "advanced": "Morphological computation where the robot's physical properties contribute to intelligent behavior without explicit control"
                }
            ],
            "ros2": [
                {
                    "beginner": "Using ROS2 to make a camera and a robot arm work together to pick up objects",
                    "intermediate": "Implementing a navigation stack with multiple nodes communicating via ROS2 topics and services",
                    "advanced": "Designing a distributed robotic system with real-time constraints using ROS2's Quality of Service settings"
                }
            ]
        }

        if concept_lower in examples:
            concept_examples = examples[concept_lower][0]  # Get the first (and only) example dict
            return [concept_examples.get(level, concept_examples.get("beginner", f"Example of {concept}"))]
        else:
            return [f"Example of how {concept} is used in robotics applications"]

    def _get_related_concepts(self, concept: str) -> list:
        """Get related concepts"""
        concept_lower = concept.lower()

        related = {
            "embodied cognition": ["affordance", "active inference", "morphological computation", "sensorimotor learning"],
            "ros2": ["nodes", "topics", "services", "actions", "parameters", "dds", "qos"],
            "inverse kinematics": ["forward kinematics", "jacobian", "kinematic chain", "end effector"],
            "zero moment point": ["center of mass", "stability", "bipedal locomotion", "balance control"]
        }

        return related.get(concept_lower, [f"Related concept to {concept}"])

    def _get_mathematical_formulation(self, concept: str) -> str:
        """Get mathematical formulation if applicable"""
        concept_lower = concept.lower()

        math_forms = {
            "zero moment point": "ZMP = (M_x/M_z, M_y/M_z) where M_x, M_y, M_z are moments and forces in the respective directions",
            "inverse kinematics": "Given end-effector pose T, find joint angles θ such that f(θ) = T, where f is the forward kinematics function"
        }

        return math_forms.get(concept_lower, "")

    def _get_practical_applications(self, concept: str) -> list:
        """Get practical applications"""
        concept_lower = concept.lower()

        applications = {
            "embodied cognition": [
                "Development of adaptive robots that learn from physical interaction",
                "Design of robot bodies that enhance cognitive capabilities",
                "Creation of more natural human-robot interaction"
            ],
            "ros2": [
                "Building modular robot software systems",
                "Integrating different sensors and actuators",
                "Creating reusable robotic applications"
            ],
            "inverse kinematics": [
                "Controlling robot arms for manipulation tasks",
                "Programming humanoid robots to reach specific positions",
                "Motion planning for complex robotic systems"
            ]
        }

        return applications.get(concept_lower, [f"Application of {concept} in robotics"])


# Example usage and testing
if __name__ == "__main__":
    tool = ConceptExplanationTool()

    # Test concept explanation
    result = tool.explain_concept("embodied cognition", "beginner", True)
    print("Concept Explanation:")
    print(f"Concept: {result['concept']}")
    print(f"Definition: {result['definition']}")
    print(f"Explanation: {result['detailed_explanation']}")
    print(f"Examples: {result['examples']}")
    print(f"Related: {result['related_concepts']}")