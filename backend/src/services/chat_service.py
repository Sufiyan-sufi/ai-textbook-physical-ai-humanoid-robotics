from typing import Dict, Any, List, Optional
import logging
import json
from datetime import datetime
from openai import OpenAI
import os
from src.services.rag_service import RAGService


class ChatService:
    """Service to handle chat conversations with AI tutor"""

    def __init__(self):
        self.client = None  # Initialize OpenAI client lazily to avoid startup issues
        import os
        self.rag_service = RAGService(
            qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
            qdrant_api_key=os.getenv("QDRANT_API_KEY"),
            qdrant_cluster_url=os.getenv("QDRANT_CLUSTER_URL")
        )
        self.logger = logging.getLogger(__name__)

    def _get_openai_client(self):
        """Lazy initialization of OpenAI client"""
        if self.client is None:
            from openai import OpenAI
            import os
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY environment variable is not set")
            self.client = OpenAI(api_key=api_key)
        return self.client

    def create_conversation_context(self, user_message: str, retrieved_context: str = "",
                                  user_preferences: Optional[Dict[str, Any]] = None) -> List[Dict[str, str]]:
        """Create the conversation context for the AI model"""
        context_messages = []

        # System message with instructions for the AI tutor
        system_message = {
            "role": "system",
            "content": f"""You are an AI robotics tutor helping engineering students learn Physical AI and Humanoid Robotics.
Your responses should be:
1. Educational and accurate
2. Adapted to the student's learning level
3. Connected to the textbook content when possible
4. Clear and helpful

{"User preferences: " + json.dumps(user_preferences) if user_preferences else ""}
{"Retrieved textbook content: " + retrieved_context if retrieved_context else ""}

Always be encouraging and provide practical examples when possible."""
        }
        context_messages.append(system_message)

        return context_messages

    def chat_with_context(self, user_message: str, conversation_history: List[Dict[str, str]] = None,
                         user_preferences: Optional[Dict[str, Any]] = None,
                         textbook_query: Optional[str] = None) -> Dict[str, Any]:
        """Chat with the AI tutor using retrieved context"""
        try:
            # If no specific textbook query is provided, use the user message for RAG
            rag_query = textbook_query or user_message

            # Retrieve relevant textbook content
            rag_response = self.rag_service.query_with_reranking(rag_query, top_k=3)
            retrieved_context = rag_response.get("retrieved_context", "")
            generated_answer = rag_response.get("answer", "")

            # Create the conversation context
            context_messages = self.create_conversation_context(
                user_message, retrieved_context, user_preferences
            )

            # Add conversation history if available
            if conversation_history:
                # Limit history to prevent token overflow
                recent_history = conversation_history[-5:]  # Use last 5 exchanges
                context_messages.extend(recent_history)

            # Add the current user message
            context_messages.append({
                "role": "user",
                "content": user_message
            })

            # Get the OpenAI client (lazy initialization)
            client = self._get_openai_client()

            # Call the OpenAI API
            response = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=context_messages,
                max_tokens=1000,
                temperature=0.7,
                top_p=0.9
            )

            # Extract the response
            ai_response = response.choices[0].message.content
            usage = response.usage

            # Format the response
            result = {
                "response": ai_response,
                "usage": {
                    "prompt_tokens": usage.prompt_tokens,
                    "completion_tokens": usage.completion_tokens,
                    "total_tokens": usage.total_tokens
                },
                "model": response.model,
                "timestamp": datetime.now().isoformat(),
                "retrieved_sources": rag_response.get("results", []),
                "confidence": self._calculate_confidence(ai_response, rag_response)
            }

            self.logger.info(f"Chat completed for query: '{user_message[:50]}...' - Tokens used: {usage.total_tokens}")
            return result

        except Exception as e:
            self.logger.error(f"Error in chat_with_context: {e}")
            return {
                "response": f"I encountered an error while processing your request: {str(e)}. Could you please try again?",
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            }

    def _calculate_confidence(self, response: str, rag_response: Dict[str, Any]) -> float:
        """Calculate a confidence score based on RAG results"""
        results = rag_response.get("results", [])
        if not results:
            return 0.3  # Low confidence if no sources found

        # Calculate average relevance score
        avg_score = sum(r.get("score", 0) for r in results) / len(results)

        # Boost confidence if the response seems to reference the retrieved content
        response_lower = response.lower()
        content_references = 0

        for result in results:
            content_snippet = result.get("content", "")[:100].lower()  # First 100 chars
            if any(word in response_lower for word in content_snippet.split()[:10]):
                content_references += 1

        reference_ratio = content_references / max(len(results), 1)

        # Combine relevance score and content reference ratio
        confidence = (avg_score * 0.7) + (reference_ratio * 0.3)
        return min(confidence, 1.0)  # Cap at 1.0

    def generate_followup_questions(self, topic: str, context: str = "", count: int = 3) -> List[str]:
        """Generate follow-up questions based on the topic and context"""
        try:
            prompt = f"""Generate {count} follow-up questions about the following topic:

Topic: {topic}

Context: {context}

Generate thoughtful questions that would help a student deepen their understanding. Focus on practical applications, connections between concepts, or areas that might need clarification.

Return only the questions, one per line, without numbering or additional text."""

            client = self._get_openai_client()
            response = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=200,
                temperature=0.8
            )

            questions_text = response.choices[0].message.content
            questions = [q.strip() for q in questions_text.split('\\n') if q.strip() and not q.strip().startswith('1.') and not q.strip().startswith('2.') and not q.strip().startswith('3.')]

            # If the AI included numbering, try to extract just the questions
            if len(questions) == 0:
                import re
                questions = re.findall(r'[0-9]+\\.\\s*(.+)', questions_text)

            return questions[:count]

        except Exception as e:
            self.logger.error(f"Error generating follow-up questions: {e}")
            # Return some default follow-up questions
            return [
                f"What are the practical applications of {topic}?",
                f"How does {topic} relate to other concepts in robotics?",
                f"What are some common challenges when implementing {topic}?"
            ]

    def explain_complex_topic(self, topic: str, user_level: str = "intermediate",
                            examples: bool = True) -> Dict[str, Any]:
        """Provide a detailed explanation of a complex topic"""
        try:
            # Retrieve relevant content about the topic
            rag_response = self.rag_service.query(topic, top_k=5)
            retrieved_content = rag_response.get("retrieved_context", "")
            generated_answer = rag_response.get("answer", "")

            prompt_parts = [
                f"Explain the robotics concept '{topic}' to a {user_level} level student.",
                f"Use the following textbook content as reference: {retrieved_content}" if retrieved_content else "",
                "Provide a clear explanation with:" if examples else "Provide a clear explanation with:",
                "- Clear definition and key components",
                "- Practical examples or applications" if examples else "",
                "- Common misconceptions to avoid",
                "- Connections to other robotics concepts",
                "Keep the explanation educational and accessible."
            ]

            prompt = " ".join([part for part in prompt_parts if part])

            client = self._get_openai_client()
            response = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=800,
                temperature=0.6
            )

            explanation = response.choices[0].message.content

            return {
                "topic": topic,
                "explanation": explanation,
                "retrieved_sources": rag_response.get("results", []),
                "user_level": user_level,
                "examples_included": examples,
                "timestamp": datetime.now().isoformat()
            }

        except Exception as e:
            self.logger.error(f"Error explaining complex topic {topic}: {e}")
            return {
                "topic": topic,
                "explanation": f"I'm sorry, I encountered an error while explaining '{topic}'. Please try asking about this topic directly in our chat.",
                "retrieved_sources": [],
                "user_level": user_level,
                "examples_included": examples,
                "timestamp": datetime.now().isoformat(),
                "error": str(e)
            }

    def personalize_response(self, content: str, user_preferences: Dict[str, Any]) -> str:
        """Personalize content based on user preferences"""
        try:
            preferences_str = json.dumps(user_preferences)

            prompt = f"""Personalize the following educational content based on these user preferences:

User Preferences: {preferences_str}

Content to personalize: {content}

Return the personalized content that adapts to the user's learning style, preferences, and needs."""

            client = self._get_openai_client()
            response = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=1000,
                temperature=0.5
            )

            personalized_content = response.choices[0].message.content
            return personalized_content

        except Exception as e:
            self.logger.error(f"Error personalizing response: {e}")
            # Return original content if personalization fails
            return content

    def validate_response_accuracy(self, response: str, original_query: str,
                                 retrieved_context: str) -> Dict[str, Any]:
        """Validate that the response is accurate and relevant"""
        try:
            prompt = f"""Validate the following AI response for accuracy and relevance:

Original Query: {original_query}

Retrieved Context: {retrieved_context}

AI Response: {response}

Evaluate:
1. Is the response factually accurate?
2. Is it relevant to the query?
3. Does it align with the provided context?
4. Are there any hallucinations or incorrect information?

Return your evaluation as a JSON object with these fields:
- 'is_accurate': boolean
- 'is_relevant': boolean
- 'has_hallucinations': boolean
- 'confidence_score': number between 0 and 1
- 'feedback': string with specific feedback"""

            client = self._get_openai_client()
            validation_response = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.2,
                response_format={"type": "json_object"}
            )

            import json
            validation_result = json.loads(validation_response.choices[0].message.content)

            return validation_result

        except Exception as e:
            self.logger.error(f"Error validating response: {e}")
            # Return a default validation result
            return {
                "is_accurate": True,
                "is_relevant": True,
                "has_hallucinations": False,
                "confidence_score": 0.8,
                "feedback": "Validation could not be performed due to an error."
            }