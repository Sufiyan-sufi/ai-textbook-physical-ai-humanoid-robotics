import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, AsyncMock
from src.main import app
from src.api.v1.ai_tutor_routes import chat, query


@pytest.fixture
def client():
    """Create test client for FastAPI app"""
    return TestClient(app)


@pytest.mark.asyncio
async def test_chat_endpoint_success(client):
    """Test the chat endpoint with valid input"""
    with patch('src.api.v1.ai_tutor_routes.ChatService') as mock_chat_service_class:
        # Mock the chat service instance
        mock_chat_service = AsyncMock()
        mock_chat_service.chat.return_value = {
            "message": "This is a test response",
            "sources": ["source1"],
            "tools_used": []
        }
        mock_chat_service_class.return_value = mock_chat_service

        response = client.post(
            "/api/v1/ai-tutor/chat",
            json={"message": "Hello", "context": {}}
        )

        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert data["message"] == "This is a test response"


@pytest.mark.asyncio
async def test_chat_endpoint_missing_message(client):
    """Test the chat endpoint with missing message"""
    response = client.post(
        "/api/v1/ai-tutor/chat",
        json={"context": {}}
    )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_query_endpoint_success(client):
    """Test the query endpoint with valid input"""
    with patch('src.api.v1.ai_tutor_routes.RAGService') as mock_rag_service_class:
        # Mock the RAG service instance
        mock_rag_service = AsyncMock()
        mock_rag_service.query.return_value = {
            "results": ["result1", "result2"],
            "query": "test query",
            "context": {}
        }
        mock_rag_service_class.return_value = mock_rag_service

        response = client.post(
            "/api/v1/ai-tutor/query",
            json={"query": "What is ROS 2?", "context": {}}
        )

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) == 2


@pytest.mark.asyncio
async def test_query_endpoint_missing_query(client):
    """Test the query endpoint with missing query"""
    response = client.post(
        "/api/v1/ai-tutor/query",
        json={"context": {}}
    )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_chat_endpoint_service_error(client):
    """Test the chat endpoint when service raises an error"""
    with patch('src.api.v1.ai_tutor_routes.ChatService') as mock_chat_service_class:
        # Mock the chat service to raise an exception
        mock_chat_service = Mock()
        mock_chat_service.chat = AsyncMock(side_effect=Exception("Service error"))
        mock_chat_service_class.return_value = mock_chat_service

        response = client.post(
            "/api/v1/ai-tutor/chat",
            json={"message": "Hello", "context": {}}
        )

        # Should return 500 for internal server error
        assert response.status_code == 500


@pytest.mark.asyncio
async def test_query_endpoint_service_error(client):
    """Test the query endpoint when service raises an error"""
    with patch('src.api.v1.ai_tutor_routes.RAGService') as mock_rag_service_class:
        # Mock the RAG service to raise an exception
        mock_rag_service = Mock()
        mock_rag_service.query = AsyncMock(side_effect=Exception("Service error"))
        mock_rag_service_class.return_value = mock_rag_service

        response = client.post(
            "/api/v1/ai-tutor/query",
            json={"query": "What is ROS 2?", "context": {}}
        )

        # Should return 500 for internal server error
        assert response.status_code == 500


def test_chat_request_model_validation():
    """Test ChatRequest model validation"""
    from src.api.v1.ai_tutor_routes import ChatRequest

    # Valid request
    valid_request = ChatRequest(message="Hello", context={})
    assert valid_request.message == "Hello"
    assert valid_request.context == {}

    # Test with additional context
    context_request = ChatRequest(
        message="Hello",
        context={"user_id": "test_user", "chapter": "intro"}
    )
    assert context_request.context["user_id"] == "test_user"


def test_query_request_model_validation():
    """Test QueryRequest model validation"""
    from src.api.v1.ai_tutor_routes import QueryRequest

    # Valid request
    valid_request = QueryRequest(query="What is ROS 2?", context={})
    assert valid_request.query == "What is ROS 2?"
    assert valid_request.context == {}


if __name__ == "__main__":
    pytest.main([__file__])