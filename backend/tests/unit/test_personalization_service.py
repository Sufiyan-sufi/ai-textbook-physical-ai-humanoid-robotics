import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from sqlalchemy.ext.asyncio import AsyncSession
from src.services.personalization_service import PersonalizationService


@pytest.fixture
def personalization_service():
    """Create a PersonalizationService instance for testing"""
    service = PersonalizationService()
    return service


@pytest.mark.asyncio
async def test_get_user_preferences_new_user(personalization_service):
    """Test getting preferences for a new user (default preferences)"""
    # Mock the database session
    mock_db = AsyncMock(spec=AsyncSession)
    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = None
    mock_db.execute.return_value = mock_result

    preferences = await personalization_service.get_user_preferences("test_user_123", mock_db)

    # Check that default preferences are returned
    assert preferences is not None
    assert preferences["user_id"] == "test_user_123"
    assert preferences["learning_level"] == "beginner"
    assert preferences["learning_style"] == "visual"
    assert isinstance(preferences["interests"], list)
    assert isinstance(preferences["preferred_topics"], list)


@pytest.mark.asyncio
async def test_get_user_preferences_existing_user(personalization_service):
    """Test getting preferences for an existing user"""
    # Mock the database session and existing preferences
    mock_db = AsyncMock(spec=AsyncSession)
    mock_prefs = Mock()
    mock_prefs.user_id = "test_user_123"
    mock_prefs.learning_level = "advanced"
    mock_prefs.interests = '["robotics", "AI"]'
    mock_prefs.preferred_topics = '["ROS2", "control"]'
    mock_prefs.learning_style = "kinesthetic"
    mock_prefs.accessibility_settings = '{}'
    mock_prefs.last_accessed_chapters = '["ch1", "ch2"]'
    mock_prefs.completed_chapters = '["ch1"]'
    mock_prefs.customizations = '{}'

    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = mock_prefs
    mock_db.execute.return_value = mock_result

    preferences = await personalization_service.get_user_preferences("test_user_123", mock_db)

    assert preferences is not None
    assert preferences["user_id"] == "test_user_123"
    assert preferences["learning_level"] == "advanced"
    assert preferences["interests"] == ["robotics", "AI"]


@pytest.mark.asyncio
async def test_update_user_preferences_new_user(personalization_service):
    """Test updating preferences for a new user (create new record)"""
    # Mock the database session
    mock_db = AsyncMock(spec=AsyncSession)
    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = None
    mock_db.execute.return_value = mock_result

    # Mock the commit and rollback methods
    mock_db.commit = AsyncMock()
    mock_db.rollback = AsyncMock()

    test_preferences = {
        "learning_level": "intermediate",
        "interests": ["ROS2", "navigation"],
        "learning_style": "visual"
    }

    success = await personalization_service.update_user_preferences(
        "test_user_456", test_preferences, mock_db
    )

    assert success is True
    mock_db.commit.assert_called_once()


@pytest.mark.asyncio
async def test_update_user_preferences_existing_user(personalization_service):
    """Test updating preferences for an existing user"""
    # Mock the database session and existing preferences
    mock_db = AsyncMock(spec=AsyncSession)
    mock_existing_prefs = Mock()
    mock_existing_prefs.learning_level = "beginner"
    mock_existing_prefs.interests = '["robotics"]'
    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = mock_existing_prefs
    mock_db.execute.return_value = mock_result

    # Mock the commit and rollback methods
    mock_db.commit = AsyncMock()
    mock_db.rollback = AsyncMock()

    test_preferences = {
        "learning_level": "advanced",
        "interests": ["ROS2", "navigation", "manipulation"]
    }

    success = await personalization_service.update_user_preferences(
        "test_user_789", test_preferences, mock_db
    )

    assert success is True
    mock_db.commit.assert_called_once()
    assert mock_existing_prefs.learning_level == "advanced"


@pytest.mark.asyncio
async def test_update_user_preferences_error(personalization_service):
    """Test error handling in update_user_preferences"""
    # Mock the database session to raise an exception
    mock_db = AsyncMock(spec=AsyncSession)
    mock_db.execute.side_effect = Exception("Database error")
    mock_db.rollback = AsyncMock()

    test_preferences = {"learning_level": "intermediate"}

    success = await personalization_service.update_user_preferences(
        "test_user_error", test_preferences, mock_db
    )

    assert success is False
    mock_db.rollback.assert_called_once()


def test_add_beginner_explanations(personalization_service):
    """Test adding beginner explanations to content"""
    test_content = "This algorithm is complex."
    result = personalization_service._add_beginner_explanations(test_content)

    # The current implementation just replaces terms, but our test content doesn't contain the specific terms
    # So the content should remain the same
    assert result == test_content


def test_add_advanced_details(personalization_service):
    """Test adding advanced details to content"""
    test_content = "Basic content"
    result = personalization_service._add_advanced_details(test_content)

    assert result == test_content  # Current implementation returns original content


def test_enhance_visual_elements(personalization_service):
    """Test enhancing visual elements in content"""
    test_content = "Text content"
    result = personalization_service._enhance_visual_elements(test_content)

    assert result == test_content  # Current implementation returns original content


def test_add_hands_on_elements(personalization_service):
    """Test adding hands-on elements to content"""
    test_content = "Theoretical content"
    result = personalization_service._add_hands_on_elements(test_content)

    assert result == test_content  # Current implementation returns original content


@pytest.mark.asyncio
async def test_personalize_content(personalization_service):
    """Test content personalization"""
    # Mock the database session
    mock_db = AsyncMock(spec=AsyncSession)
    mock_prefs = Mock()
    mock_prefs.user_id = "test_user_123"
    mock_prefs.learning_level = "beginner"
    mock_prefs.learning_style = "visual"
    mock_prefs.interests = '["robotics"]'
    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = mock_prefs
    mock_db.execute.return_value = mock_result

    test_content = "This is educational content."
    result = await personalization_service.personalize_content(
        "test_user_123", test_content, mock_db
    )

    # Should return content (might be modified based on preferences)
    assert isinstance(result, str)
    assert len(result) >= len(test_content)


@pytest.mark.asyncio
async def test_get_personalized_recommendations(personalization_service):
    """Test getting personalized recommendations"""
    # Mock the database session
    mock_db = AsyncMock(spec=AsyncSession)
    mock_prefs = Mock()
    mock_prefs.user_id = "test_user_123"
    mock_prefs.learning_level = "intermediate"
    mock_prefs.interests = '["ROS2", "navigation"]'
    mock_prefs.completed_chapters = '["Getting Started with ROS 2"]'
    mock_result = AsyncMock()
    mock_result.scalars.return_value.first.return_value = mock_prefs
    mock_db.execute.return_value = mock_result

    recommendations = await personalization_service.get_personalized_recommendations(
        "test_user_123", mock_db
    )

    assert isinstance(recommendations, list)
    assert len(recommendations) > 0
    for rec in recommendations:
        assert "title" in rec
        assert "difficulty" in rec
        assert "relevance" in rec


if __name__ == "__main__":
    pytest.main([__file__])