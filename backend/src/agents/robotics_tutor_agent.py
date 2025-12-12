from typing import Dict, Any, List
from pydantic import BaseModel
from openai import OpenAI
import json
import os
from src.services.rag_service import RAGService
from src.services.chat_service import ChatService
from src.agents.tools.code_explanation_tool import CodeExplanationTool
from src.agents.tools.concept_explanation_tool import ConceptExplanationTool
from src.agents.tools.simulation_tool import SimulationTool


class AgentResponse(BaseModel):
    """Response model for agent interactions"""
    message: str
    tool_calls: List[Dict[str, Any]] = []
    context: Dict[str, Any] = {}


class RoboticsTutorAgent:
    """AI agent specialized for robotics tutoring using ChatKit"""

    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.rag_service = RAGService()
        self.chat_service = ChatService()
        self.code_explanation_tool = CodeExplanationTool()
        self.concept_explanation_tool = ConceptExplanationTool()
        self.simulation_tool = SimulationTool()

    def get_tools(self):
        """Return available tools for the agent"""
        return [
            {
                "type": "function",
                "function": {
                    "name": "explain_code",
                    "description": "Explain ROS 2 code concepts and provide debugging help",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "code_snippet": {"type": "string", "description": "The code snippet to explain"},
                            "language": {"type": "string", "description": "Programming language (python, cpp, etc.)"},
                            "context": {"type": "string", "description": "Additional context about the code"}
                        },
                        "required": ["code_snippet", "language"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "explain_concept",
                    "description": "Explain robotics concepts from the textbook content",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "concept": {"type": "string", "description": "The robotics concept to explain"},
                            "level": {"type": "string", "description": "Difficulty level: beginner, intermediate, advanced"},
                            "examples": {"type": "boolean", "description": "Whether to include practical examples"}
                        },
                        "required": ["concept"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "run_simulation",
                    "description": "Run a simulation for the student to demonstrate concepts",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "scenario": {"type": "string", "description": "Type of simulation to run"},
                            "parameters": {"type": "object", "description": "Simulation parameters"},
                            "environment": {"type": "string", "description": "Target environment: gazebo, isaac, unity"}
                        },
                        "required": ["scenario", "environment"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "search_knowledge_base",
                    "description": "Search the textbook content for relevant information",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "Search query for textbook content"},
                            "context": {"type": "string", "description": "Additional context for the search"}
                        },
                        "required": ["query"]
                    }
                }
            }
        ]

    def process_message(self, user_message: str, user_context: Dict[str, Any] = None) -> AgentResponse:
        """Process a user message and return an agent response"""
        try:
            # Prepare the system message with agent instructions
            system_message = {
                "role": "system",
                "content": """You are an AI robotics tutor helping engineering students learn Physical AI and Humanoid Robotics.
                Use the available tools to explain concepts, debug code, and demonstrate simulations.
                Be patient, educational, and provide practical examples when possible.
                Always maintain technical accuracy and connect concepts to the textbook content."""
            }

            # Prepare the conversation history
            messages = [system_message]

            # Add user context if available
            if user_context:
                context_message = {
                    "role": "system",
                    "content": f"User context: {json.dumps(user_context)}"
                }
                messages.append(context_message)

            # Add the user message
            messages.append({
                "role": "user",
                "content": user_message
            })

            # Call the OpenAI API with tools
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=messages,
                tools=self.get_tools(),
                tool_choice="auto"
            )

            # Process the response
            choice = response.choices[0]
            message = choice.message

            # Extract tool calls if any
            tool_calls = []
            if message.tool_calls:
                for tool_call in message.tool_calls:
                    tool_calls.append({
                        "id": tool_call.id,
                        "type": tool_call.type,
                        "function": {
                            "name": tool_call.function.name,
                            "arguments": tool_call.function.arguments
                        }
                    })

            # If there's content in the message, return it
            if message.content:
                return AgentResponse(
                    message=message.content,
                    tool_calls=tool_calls,
                    context={"model": "gpt-4-turbo", "finish_reason": choice.finish_reason}
                )
            else:
                # If there are tool calls but no content, return tool calls
                return AgentResponse(
                    message="Processing your request with specialized tools...",
                    tool_calls=tool_calls,
                    context={"model": "gpt-4-turbo", "finish_reason": choice.finish_reason}
                )

        except Exception as e:
            return AgentResponse(
                message=f"I encountered an error while processing your request: {str(e)}. Could you please rephrase your question?",
                tool_calls=[],
                context={"error": str(e)}
            )

    def execute_tool_call(self, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a specific tool call with given arguments"""
        try:
            if tool_name == "explain_code":
                return self.code_explanation_tool.explain_code(
                    code_snippet=arguments.get("code_snippet", ""),
                    language=arguments.get("language", "python"),
                    context=arguments.get("context", "")
                )
            elif tool_name == "explain_concept":
                return self.concept_explanation_tool.explain_concept(
                    concept=arguments.get("concept", ""),
                    level=arguments.get("level", "beginner"),
                    examples=arguments.get("examples", True)
                )
            elif tool_name == "run_simulation":
                return self.simulation_tool.run_simulation(
                    scenario=arguments.get("scenario", ""),
                    parameters=arguments.get("parameters", {}),
                    environment=arguments.get("environment", "gazebo")
                )
            elif tool_name == "search_knowledge_base":
                query = arguments.get("query", "")
                context = arguments.get("context", "")
                # Use RAG service to search textbook content
                results = self.rag_service.query(query, context)
                return {
                    "status": "success",
                    "results": results,
                    "query": query
                }
            else:
                return {
                    "status": "error",
                    "message": f"Unknown tool: {tool_name}"
                }
        except Exception as e:
            return {
                "status": "error",
                "message": f"Error executing tool {tool_name}: {str(e)}"
            }