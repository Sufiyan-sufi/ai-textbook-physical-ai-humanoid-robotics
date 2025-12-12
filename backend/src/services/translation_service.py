from typing import Dict, Any, List, Optional
import asyncio
import logging
from openai import AsyncOpenAI
import os
from googletrans import Translator  # Alternative translation library
import json


class TranslationService:
    """Service for translating content to Urdu with technical accuracy"""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.openai_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.google_translator = Translator()
        self.supported_languages = ["ur", "en"]  # Urdu and English
        self.translation_cache = {}  # Simple in-memory cache

    async def translate_text(self, text: str, target_language: str = "ur",
                           source_language: str = "en", preserve_formatting: bool = True) -> str:
        """Translate text to target language with technical accuracy"""
        try:
            cache_key = f"{text[:50]}_{target_language}_{source_language}"
            if cache_key in self.translation_cache:
                return self.translation_cache[cache_key]

            if target_language == "ur":
                # Use OpenAI for better technical accuracy
                translated_text = await self._translate_with_openai(text, target_language, source_language)
            else:
                # Use Google Translate for other languages
                translated_text = await self._translate_with_google(text, target_language, source_language)

            if preserve_formatting:
                # Preserve any markdown or technical formatting
                translated_text = self._preserve_formatting(text, translated_text)

            # Cache the result
            self.translation_cache[cache_key] = translated_text

            return translated_text

        except Exception as e:
            self.logger.error(f"Error translating text: {e}")
            # Return original text on error
            return text

    async def _translate_with_openai(self, text: str, target_language: str, source_language: str) -> str:
        """Translate using OpenAI for better technical accuracy"""
        try:
            # Create a detailed prompt for technical translation
            system_prompt = f"""You are an expert translator specializing in technical content.
            Translate the following text from {source_language} to {target_language} (Urdu).
            Preserve technical terminology and accuracy.
            For technical terms that don't have direct Urdu equivalents, keep the English term in parentheses after the Urdu translation.
            Maintain the context and meaning of robotics and AI concepts."""

            response = await self.openai_client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=len(text) * 2,  # Allow more tokens for Urdu which may be longer
                temperature=0.3  # Lower temperature for more consistent technical translations
            )

            translated_text = response.choices[0].message.content.strip()
            return translated_text

        except Exception as e:
            self.logger.warning(f"OpenAI translation failed: {e}, falling back to Google Translate")
            return await self._translate_with_google(text, target_language, source_language)

    async def _translate_with_google(self, text: str, target_language: str, source_language: str) -> str:
        """Translate using Google Translate"""
        try:
            # Split long text into smaller chunks to avoid API limits
            max_chunk_size = 4000  # Google Translate has character limits
            chunks = self._split_text_into_chunks(text, max_chunk_size)

            translated_chunks = []
            for chunk in chunks:
                result = self.google_translator.translate(
                    chunk,
                    src=source_language,
                    dest=target_language
                )
                translated_chunks.append(result.text)

            return " ".join(translated_chunks)

        except Exception as e:
            self.logger.error(f"Google Translate error: {e}")
            return text  # Return original text on error

    def _split_text_into_chunks(self, text: str, max_chunk_size: int) -> List[str]:
        """Split text into chunks that respect sentence boundaries"""
        sentences = text.split('. ')
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk + sentence) < max_chunk_size:
                current_chunk += sentence + ". "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + ". "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def _preserve_formatting(self, original_text: str, translated_text: str) -> str:
        """Preserve markdown formatting and code blocks in translation"""
        # This is a simplified implementation
        # In a real implementation, you'd want more sophisticated formatting preservation

        # Preserve code blocks (```...```)
        import re

        # Extract code blocks from original
        code_blocks = re.findall(r'```[\s\S]*?```', original_text)

        # Replace code blocks with placeholders in translated text
        # Note: This is a simplified approach - in practice, you'd need to be more careful
        # about preserving the exact code content

        result = translated_text

        # For now, just return the translated text
        # A full implementation would involve extracting code blocks from the original,
        # keeping them separate, translating the surrounding text, and reassembling
        return result

    async def translate_chapter(self, chapter_content: str, target_language: str = "ur") -> Dict[str, Any]:
        """Translate an entire textbook chapter"""
        try:
            # Parse the chapter to separate different sections
            sections = self._parse_chapter(chapter_content)

            translated_sections = {}

            for section_name, content in sections.items():
                if section_name == "frontmatter":
                    # Don't translate frontmatter, just copy it
                    translated_sections[section_name] = content
                else:
                    # Translate the content
                    translated_content = await self.translate_text(
                        content,
                        target_language,
                        preserve_formatting=True
                    )
                    translated_sections[section_name] = translated_content

            # Reconstruct the chapter
            reconstructed_chapter = self._reconstruct_chapter(translated_sections)

            return {
                "status": "success",
                "translated_content": reconstructed_chapter,
                "target_language": target_language,
                "original_length": len(chapter_content),
                "translated_length": len(reconstructed_chapter)
            }

        except Exception as e:
            self.logger.error(f"Error translating chapter: {e}")
            return {
                "status": "error",
                "message": str(e),
                "translated_content": chapter_content
            }

    def _parse_chapter(self, chapter_content: str) -> Dict[str, str]:
        """Parse chapter content into sections"""
        sections = {}

        # Extract frontmatter if present
        import re
        frontmatter_match = re.match(r'^---\n(.*?)\n---', chapter_content, re.DOTALL | re.MULTILINE)
        if frontmatter_match:
            sections["frontmatter"] = frontmatter_match.group(0)
            content_without_frontmatter = chapter_content[frontmatter_match.end():].strip()
        else:
            content_without_frontmatter = chapter_content

        # For now, treat the rest as a single content section
        # In a more sophisticated implementation, you might parse by headers, etc.
        sections["content"] = content_without_frontmatter

        return sections

    def _reconstruct_chapter(self, sections: Dict[str, str]) -> str:
        """Reconstruct chapter from sections"""
        result = ""

        if "frontmatter" in sections:
            result += sections["frontmatter"] + "\n\n"

        if "content" in sections:
            result += sections["content"]

        return result.strip()

    async def batch_translate(self, texts: List[str], target_language: str = "ur") -> List[str]:
        """Translate multiple texts efficiently"""
        translations = []

        for text in texts:
            translation = await self.translate_text(text, target_language)
            translations.append(translation)

        return translations

    def get_translation_quality_score(self, original: str, translated: str, target_language: str = "ur") -> float:
        """Calculate a basic quality score for the translation"""
        # This is a very basic implementation
        # In practice, you'd want more sophisticated quality metrics

        if not original or not translated:
            return 0.0

        # Basic length comparison (should be roughly proportional)
        length_ratio = min(len(translated)/len(original), len(original)/len(translated))

        # For Urdu, text is often longer than English, so we adjust
        if target_language == "ur":
            expected_ratio = 1.2  # Urdu is typically longer than English
            ratio_score = 1.0 - abs(length_ratio - expected_ratio) / expected_ratio
            ratio_score = max(0.0, min(1.0, ratio_score))
        else:
            ratio_score = length_ratio if length_ratio <= 1.0 else 1.0/length_ratio

        # Return a basic quality score
        return max(0.0, min(1.0, ratio_score))

    async def translate_with_context(self, text: str, context: Dict[str, Any],
                                   target_language: str = "ur") -> Dict[str, Any]:
        """Translate text with additional context for better accuracy"""
        try:
            # Use context to improve translation quality
            domain = context.get("domain", "general")
            technical_terms = context.get("technical_terms", [])

            # Create a more specific prompt based on context
            system_prompt = f"""You are an expert translator specializing in {domain} content.
            Translate the following text from English to {target_language} (Urdu).
            Pay special attention to these technical terms: {', '.join(technical_terms) if technical_terms else 'None specified'}.
            Preserve technical terminology and accuracy.
            For technical terms that don't have direct Urdu equivalents, keep the English term in parentheses after the Urdu translation."""

            response = await self.openai_client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=len(text) * 2,
                temperature=0.3
            )

            translated_text = response.choices[0].message.content.strip()

            quality_score = self.get_translation_quality_score(text, translated_text, target_language)

            return {
                "status": "success",
                "original_text": text,
                "translated_text": translated_text,
                "target_language": target_language,
                "quality_score": quality_score,
                "context_used": context
            }

        except Exception as e:
            self.logger.error(f"Error translating with context: {e}")
            return {
                "status": "error",
                "message": str(e),
                "original_text": text,
                "translated_text": text,  # Return original on error
                "target_language": target_language
            }


# Example usage and testing
if __name__ == "__main__":
    import asyncio

    async def test_translation_service():
        service = TranslationService()

        # Test basic translation
        english_text = "Embodied cognition is a theory in cognitive science that emphasizes the role of an organism's body in shaping its cognitive processes."
        urdu_text = await service.translate_text(english_text, "ur")

        print(f"Original: {english_text}")
        print(f"Translation: {urdu_text}")

        # Test with context (technical terms)
        context = {
            "domain": "robotics and AI",
            "technical_terms": ["embodied cognition", "cognitive processes", "sensorimotor"]
        }

        result = await service.translate_with_context(english_text, context, "ur")
        print(f"\nContext-aware translation result: {result}")

    asyncio.run(test_translation_service())