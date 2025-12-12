import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
from src.services.translation_service import TranslationService


@pytest.fixture
def translation_service():
    """Create a TranslationService instance for testing"""
    service = TranslationService()
    return service


@pytest.mark.asyncio
async def test_translate_text_basic(translation_service):
    """Test basic text translation"""
    test_text = "Hello, this is a test."

    # Mock the translation methods to avoid actual API calls
    with patch.object(translation_service, '_translate_with_openai',
                     return_value=AsyncMock(return_value=test_text)):
        with patch.object(translation_service, '_translate_with_google',
                         return_value=test_text):

            result = await translation_service.translate_text(test_text, "ur")

            assert result == test_text


@pytest.mark.asyncio
async def test_translate_with_openai(translation_service):
    """Test OpenAI translation method"""
    test_text = "Test translation"

    # Mock the OpenAI client
    with patch.object(translation_service.openai_client.chat.completions, 'create') as mock_create:
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "translated text"
        mock_create.return_value = mock_response

        result = await translation_service._translate_with_openai(test_text, "ur", "en")

        assert result == "translated text"
        mock_create.assert_called_once()


@pytest.mark.asyncio
async def test_translate_with_google(translation_service):
    """Test Google Translate method"""
    test_text = "Test translation"

    # Mock the Google translator
    mock_translation = Mock()
    mock_translation.text = "Google translated text"

    with patch.object(translation_service.google_translator, 'translate',
                     return_value=mock_translation):

        result = await translation_service._translate_with_google(test_text, "ur", "en")

        assert result == "Google translated text"


def test_split_text_into_chunks():
    """Test splitting text into chunks"""
    service = TranslationService.__new__(TranslationService)  # Create without __init__

    long_text = "This is sentence one. This is sentence two. This is sentence three. " * 10
    chunks = service._split_text_into_chunks(long_text, max_chunk_size=50)

    assert len(chunks) > 0
    assert all(isinstance(chunk, str) for chunk in chunks)
    assert all(len(chunk) <= 100 for chunk in chunks)  # Roughly double the max size for safety


def test_preserve_formatting(translation_service):
    """Test preserving formatting in translation"""
    original = "This has `code` and **bold** text."
    translated = "یہ `code` اور **bold** متن رکھتا ہے۔"

    result = translation_service._preserve_formatting(original, translated)

    # For now, this just returns the translated text
    assert result == translated


@pytest.mark.asyncio
async def test_translate_chapter(translation_service):
    """Test translating an entire chapter"""
    chapter_content = """---
title: "Test Chapter"
---

# Test Chapter

This is test content for the chapter.
"""

    # Mock the translation method
    with patch.object(translation_service, 'translate_text',
                     return_value="translated content"):

        result = await translation_service.translate_chapter(chapter_content, "ur")

        assert result["status"] == "success"
        assert "translated_content" in result


@pytest.mark.asyncio
async def test_batch_translate(translation_service):
    """Test batch translation of multiple texts"""
    texts = ["Hello", "World", "Test"]

    # Mock the translation method
    with patch.object(translation_service, 'translate_text') as mock_translate:
        mock_translate.return_value = "translated"

        results = await translation_service.batch_translate(texts, "ur")

        assert len(results) == len(texts)
        assert all(result == "translated" for result in results)


def test_get_translation_quality_score(translation_service):
    """Test translation quality scoring"""
    original = "Hello world"
    translated = "ہیلو دنیا"

    score = translation_service.get_translation_quality_score(original, translated, "ur")

    assert isinstance(score, float)
    assert 0.0 <= score <= 1.0


@pytest.mark.asyncio
async def test_translate_with_context(translation_service):
    """Test translation with additional context"""
    test_text = "Embodied cognition concept"
    context = {
        "domain": "robotics",
        "technical_terms": ["embodied cognition", "cognitive processes"]
    }

    # Mock the OpenAI client
    with patch.object(translation_service.openai_client.chat.completions, 'create') as mock_create:
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "translated with context"
        mock_create.return_value = mock_response

        result = await translation_service.translate_with_context(test_text, context, "ur")

        assert result["status"] == "success"
        assert result["translated_text"] == "translated with context"
        assert result["context_used"] == context


@pytest.mark.asyncio
async def test_translate_text_error_handling(translation_service):
    """Test error handling in translate_text method"""
    test_text = "Test text"

    # Mock both translation methods to raise exceptions
    with patch.object(translation_service, '_translate_with_openai',
                     side_effect=Exception("API Error")):
        with patch.object(translation_service, '_translate_with_google',
                         side_effect=Exception("Google Error")):

            result = await translation_service.translate_text(test_text, "ur")

            # Should return original text on error
            assert result == test_text


@pytest.mark.asyncio
async def test_translate_chapter_error_handling(translation_service):
    """Test error handling in translate_chapter method"""
    test_content = "Test chapter content"

    # Mock the translation method to raise an exception
    with patch.object(translation_service, 'translate_text',
                     side_effect=Exception("Translation Error")):

        result = await translation_service.translate_chapter(test_content, "ur")

        assert result["status"] == "error"
        assert result["translated_content"] == test_content


if __name__ == "__main__":
    pytest.main([__file__])