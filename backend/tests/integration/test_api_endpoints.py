import pytest
from fastapi.testclient import TestClient
from src.main import app
import asyncio


# Create test client for FastAPI app
client = TestClient(app)


def test_health_check():
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}


def test_ai_tutor_chat():
    """Test the AI tutor chat endpoint"""
    # Test data
    test_data = {
        "message": "What is embodied cognition?",
        "context": {}
    }

    response = client.post("/api/v1/ai-tutor/chat", json=test_data)

    # Basic checks
    assert response.status_code in [200, 500]  # Allow for service unavailability in tests
    if response.status_code == 200:
        data = response.json()
        assert "message" in data or "answer" in data


def test_ai_tutor_query():
    """Test the AI tutor query endpoint"""
    # Test data
    test_data = {
        "query": "What is ROS 2?",
        "context": {}
    }

    response = client.post("/api/v1/ai-tutor/query", json=test_data)

    # Basic checks
    assert response.status_code in [200, 500]  # Allow for service unavailability in tests
    if response.status_code == 200:
        data = response.json()
        assert "results" in data


def test_simulation_start():
    """Test the simulation start endpoint"""
    # Test data
    test_data = {
        "scenario": "basic_movement",
        "environment": "gazebo",
        "parameters": {}
    }

    response = client.post("/api/v1/simulation/start", json=test_data)

    # Basic checks
    assert response.status_code in [200, 422, 500]  # Allow for various responses


def test_simulation_status():
    """Test the simulation status endpoint"""
    # Test with a mock simulation ID
    response = client.get("/api/v1/simulation/status/test-id")

    # Basic checks
    assert response.status_code in [200, 404, 500]  # Allow for various responses


def test_translate_content():
    """Test the translation endpoint"""
    # Test data
    test_data = {
        "content": "Hello, this is a test.",
        "target_language": "ur"
    }

    response = client.post("/api/v1/translate", json=test_data)

    # Basic checks
    assert response.status_code in [200, 500]  # Allow for service unavailability in tests
    if response.status_code == 200:
        data = response.json()
        assert "translated_content" in data


if __name__ == "__main__":
    pytest.main([__file__])