from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import asyncio
import logging
import os

from src.services.rag_service import RAGService
from src.services.chat_service import ChatService

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services
rag_service = RAGService(
    qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    qdrant_api_key=os.getenv("QDRANT_API_KEY"),
    qdrant_cluster_url=os.getenv("QDRANT_CLUSTER_URL")
)
chat_service = ChatService()

class ChatRequest(BaseModel):
    user_id: Optional[str] = None
    message: str
    context: Optional[Dict[str, Any]] = None
    chapter_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
    confidence: float
    timestamp: str

class QueryRequest(BaseModel):
    query: str
    user_id: Optional[str] = None
    chapter_id: Optional[str] = None
    max_results: int = 5

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    confidence: float
    timestamp: str

@router.post("/chat", response_model=ChatResponse)
async def chat_with_ai_tutor(request: ChatRequest):
    """Chat with AI tutor using RAG to answer questions from textbook content"""
    try:
        import datetime

        # Use the chat service to handle the conversation
        chat_result = chat_service.chat_with_context(
            user_message=request.message,
            user_preferences=request.context
        )

        # Extract the response components
        response_text = chat_result.get("response", "I'm sorry, I couldn't process your request at the moment.")
        sources = [result.get("content_id", "unknown") for result in chat_result.get("retrieved_sources", [])[:5]]
        confidence = chat_result.get("confidence", 0.5)

        return ChatResponse(
            response=response_text,
            sources=sources,
            confidence=confidence,
            timestamp=chat_result.get("timestamp", datetime.datetime.now().isoformat())
        )
    except Exception as e:
        logger.error(f"Error in chat_with_ai_tutor: {e}")
        import datetime
        return ChatResponse(
            response="I encountered an error while processing your request. Please try again.",
            sources=[],
            confidence=0.0,
            timestamp=datetime.datetime.now().isoformat()
        )

@router.post("/query", response_model=QueryResponse)
async def query_textbook_content(request: QueryRequest):
    """Query textbook content using RAG for accurate answers"""
    try:
        import datetime

        # Use the RAG service to query textbook content
        rag_result = rag_service.query_with_reranking(
            query_text=request.query,
            top_k=request.max_results
        )

        # Format the results for the response
        formatted_sources = []
        for result in rag_result.get("results", []):
            formatted_sources.append({
                "content_id": result.get("content_id", ""),
                "content_preview": result.get("content", "")[:200],  # First 200 chars
                "relevance_score": result.get("score", 0.0),
                "metadata": result.get("metadata", {})
            })

        # Get the generated answer from the RAG service
        answer = rag_result.get("answer", "No relevant content found in the textbook.")

        # Calculate overall confidence based on the best result
        confidence = max([result.get("score", 0.0) for result in rag_result.get("results", [])], default=0.3)

        return QueryResponse(
            answer=answer,
            sources=formatted_sources,
            confidence=confidence,
            timestamp=datetime.datetime.now().isoformat()
        )
    except Exception as e:
        logger.error(f"Error in query_textbook_content: {e}")
        import datetime
        return QueryResponse(
            answer="I encountered an error while searching the textbook content. Please try again.",
            sources=[],
            confidence=0.0,
            timestamp=datetime.datetime.now().isoformat()
        )

@router.post("/personalize")
async def personalize_content(content: str, user_preferences: Dict[str, Any]):
    """Personalize content based on user preferences"""
    try:
        # Use the chat service to personalize the content
        personalized_content = chat_service.personalize_response(content, user_preferences)

        # For now, return the personalized content
        # In a full implementation, we might track what modifications were made
        return {
            "personalized_content": personalized_content,
            "applied_modifications": ["content_adaptation"],  # Placeholder for actual modifications
            "user_preferences_applied": user_preferences
        }
    except Exception as e:
        logger.error(f"Error in personalize_content: {e}")
        return {
            "personalized_content": content,  # Return original if personalization fails
            "applied_modifications": [],
            "error": str(e)
        }

@router.post("/translate")
async def translate_content(content: str, target_language: str = "ur"):
    """Translate content to specified language (Urdu by default)"""
    try:
        from openai import OpenAI
        import os

        client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Create a translation prompt
        prompt = f"""Translate the following technical content to {target_language}.
        Preserve technical accuracy and terminology. If {target_language} is not a valid language, translate to Urdu.

        Content to translate:
        {content}

        Translation:"""

        response = client.chat.completions.create(
            model="gpt-4-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=2000,
            temperature=0.3
        )

        translated_content = response.choices[0].message.content

        return {
            "translated_content": translated_content,
            "original_content": content,
            "target_language": target_language,
            "translation_confidence": 0.9  # Assuming high confidence for AI translation
        }
    except Exception as e:
        logger.error(f"Error in translate_content: {e}")
        return {
            "translated_content": content,  # Return original if translation fails
            "original_content": content,
            "target_language": target_language,
            "error": str(e)
        }