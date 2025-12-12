import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
from src.services.vector_service import VectorService


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing"""
    with patch('src.services.vector_service.QdrantClient') as mock:
        yield mock


@pytest.fixture
def vector_service(mock_qdrant_client):
    """Create a VectorService instance for testing"""
    service = VectorService(qdrant_url="http://test:6333", collection_name="test_collection")
    return service


@pytest.mark.asyncio
async def test_create_embedding(vector_service):
    """Test creating embeddings from text"""
    test_text = "This is a test sentence."

    # Test that embedding creation doesn't raise an error
    embedding = vector_service.create_embedding(test_text)

    # Check that embedding is a list of floats
    assert isinstance(embedding, list)
    assert len(embedding) == 384  # all-MiniLM-L6-v2 produces 384-dim vectors
    assert all(isinstance(val, float) for val in embedding)


def test_chunk_text_basic():
    """Test basic text chunking functionality"""
    service = VectorService.__new__(VectorService)  # Create without __init__ to avoid Qdrant connection

    text = "This is a test. This is only a test. In case of real emergency, this would be replaced by actual content."
    chunks = service.chunk_text(text, chunk_size=10, overlap=2)

    assert len(chunks) > 0
    assert all(isinstance(chunk, str) for chunk in chunks)
    assert all(len(chunk) > 0 for chunk in chunks)


@pytest.mark.asyncio
async def test_search_with_mock(vector_service):
    """Test search functionality with mocked Qdrant client"""
    with patch.object(vector_service.qdrant_client, 'search') as mock_search:
        # Mock search result
        mock_hit = Mock()
        mock_hit.id = "test_id"
        mock_hit.payload = {"content": "test content", "content_id": "test", "chunk_index": 0, "metadata": {}}
        mock_hit.score = 0.9
        mock_search.return_value = [mock_hit]

        results = vector_service.search("test query", limit=5)

        assert len(results) == 1
        assert results[0]["content"] == "test content"
        assert results[0]["score"] == 0.9


@pytest.mark.asyncio
async def test_index_content_with_mock(vector_service):
    """Test indexing content with mocked Qdrant client"""
    with patch.object(vector_service.qdrant_client, 'upsert') as mock_upsert:
        mock_upsert.return_value = True

        success = await vector_service.index_content(
            content_id="test_id",
            content="This is test content for indexing.",
            metadata={"test": True}
        )

        assert success is True
        mock_upsert.assert_called_once()


@pytest.mark.asyncio
async def test_delete_content_with_mock(vector_service):
    """Test deleting content with mocked Qdrant client"""
    with patch.object(vector_service.qdrant_client, 'scroll') as mock_scroll, \
         patch.object(vector_service.qdrant_client, 'delete') as mock_delete:

        # Mock scroll to return some points
        mock_point = Mock()
        mock_point.id = "test_id_0"
        mock_scroll.return_value = ([mock_point], None)

        success = vector_service.delete_content("test_id")

        assert success is True
        mock_delete.assert_called_once()


def test_get_content_chunks_with_mock(vector_service):
    """Test getting content chunks with mocked Qdrant client"""
    with patch.object(vector_service.qdrant_client, 'scroll') as mock_scroll:
        # Mock scroll result
        mock_point = Mock()
        mock_point.id = "test_id_0"
        mock_point.payload = {"content": "test content", "chunk_index": 0, "metadata": {}}
        mock_scroll.return_value = ([mock_point], None)

        chunks = vector_service.get_content_chunks("test_id")

        assert len(chunks) == 1
        assert chunks[0]["content"] == "test content"


if __name__ == "__main__":
    pytest.main([__file__])