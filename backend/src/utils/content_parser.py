from typing import Dict, Any, List
import re
import markdown
from pathlib import Path
import asyncio
import aiofiles


class ContentParser:
    """Utility for parsing and extracting content from textbook files for vectorization"""

    def __init__(self):
        self.md = markdown.Markdown(extensions=['meta', 'tables', 'fenced_code'])
        self.section_patterns = [
            r'^#{1,6}\s+(.+)$',  # Markdown headers
            r'^\d+\.\d*.*$',     # Numbered sections (1.1, 1.2, etc.)
            r'^\*\*(.+?)\*\*$',  # Bold text (often used for headings)
        ]

    def parse_mdx_content(self, content: str) -> Dict[str, Any]:
        """Parse MDX content and extract structured information"""
        # Extract frontmatter if present
        frontmatter = self._extract_frontmatter(content)

        # Remove frontmatter from content for further processing
        content_without_frontmatter = self._remove_frontmatter(content)

        # Parse the markdown content
        html_content = self.md.convert(content_without_frontmatter)

        # Extract sections and content
        sections = self._extract_sections(content_without_frontmatter)

        return {
            "frontmatter": frontmatter,
            "content": content_without_frontmatter,
            "sections": sections,
            "html": html_content,
            "word_count": len(content_without_frontmatter.split()),
            "read_time": len(content_without_frontmatter.split()) // 200  # Rough estimate
        }

    def _extract_frontmatter(self, content: str) -> Dict[str, Any]:
        """Extract frontmatter from MDX content"""
        frontmatter = {}

        # Look for YAML frontmatter between --- delimiters
        pattern = r'^---\n(.*?)\n---'
        match = re.search(pattern, content, re.DOTALL | re.MULTILINE)

        if match:
            yaml_content = match.group(1)
            lines = yaml_content.strip().split('\n')
            for line in lines:
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip().strip('"\'')
                    frontmatter[key] = value

        return frontmatter

    def _remove_frontmatter(self, content: str) -> str:
        """Remove frontmatter from content"""
        pattern = r'^---\n(.*?)\n---\n*'
        return re.sub(pattern, '', content, count=1, flags=re.DOTALL | re.MULTILINE)

    def _extract_sections(self, content: str) -> List[Dict[str, str]]:
        """Extract sections from content based on patterns"""
        sections = []
        lines = content.split('\n')

        current_section = {"title": "Introduction", "content": ""}

        for line in lines:
            # Check if this line is a section header
            is_header = False
            for pattern in self.section_patterns:
                if re.match(pattern, line.strip(), re.MULTILINE):
                    # Save the previous section
                    if current_section["content"].strip():
                        sections.append(current_section)

                    # Start a new section
                    header_match = re.match(pattern, line.strip())
                    current_section = {
                        "title": header_match.group(1).strip() if header_match else line.strip(),
                        "content": ""
                    }
                    is_header = True
                    break

            if not is_header:
                current_section["content"] += line + "\n"

        # Add the last section
        if current_section["content"].strip():
            sections.append(current_section)

        return sections

    async def parse_file(self, file_path: str) -> Dict[str, Any]:
        """Parse content from a file"""
        file_path = Path(file_path)

        if not file_path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")

        # Read file content
        async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
            content = await f.read()

        # Determine file type and parse accordingly
        if file_path.suffix.lower() in ['.md', '.mdx']:
            parsed_content = self.parse_mdx_content(content)
        elif file_path.suffix.lower() in ['.txt']:
            parsed_content = {
                "frontmatter": {},
                "content": content,
                "sections": [{"title": file_path.stem, "content": content}],
                "html": markdown.markdown(content),
                "word_count": len(content.split()),
                "read_time": len(content.split()) // 200
            }
        else:
            raise ValueError(f"Unsupported file type: {file_path.suffix}")

        # Add file metadata
        parsed_content["file_path"] = str(file_path)
        parsed_content["file_name"] = file_path.name
        parsed_content["file_type"] = file_path.suffix.lower()[1:]  # Remove the dot

        return parsed_content

    async def parse_directory(self, directory_path: str, file_extensions: List[str] = None) -> List[Dict[str, Any]]:
        """Parse all content files in a directory"""
        if file_extensions is None:
            file_extensions = ['.md', '.mdx', '.txt']

        directory = Path(directory_path)
        if not directory.exists():
            raise FileNotFoundError(f"Directory not found: {directory}")

        results = []

        for file_path in directory.rglob('*'):
            if file_path.suffix.lower() in file_extensions:
                try:
                    parsed = await self.parse_file(file_path)
                    results.append(parsed)
                except Exception as e:
                    print(f"Error parsing {file_path}: {e}")

        return results

    def clean_content(self, content: str) -> str:
        """Clean content by removing code blocks, HTML tags, and other non-text elements"""
        # Remove code blocks (both inline and fenced)
        content = re.sub(r'`[^`]*?`', '', content)  # Inline code
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)  # Fenced code blocks

        # Remove HTML tags
        content = re.sub(r'<[^>]+>', '', content)

        # Remove extra whitespace
        content = re.sub(r'\s+', ' ', content)

        return content.strip()

    def extract_key_concepts(self, content: str) -> List[str]:
        """Extract key concepts from content using simple heuristics"""
        # Look for capitalized words/phrases that might be concepts
        # This is a simple heuristic - in practice, you might use NLP techniques
        concept_patterns = [
            r'\b[A-Z][a-z]+\s+[A-Z][a-z]+\b',  # Two capitalized words
            r'\b[A-Z]{2,}\b',                   # Acronyms
            r'\b\w+[-_]\w+\b',                 # Hyphenated or underscored terms
        ]

        concepts = set()
        for pattern in concept_patterns:
            matches = re.findall(pattern, content)
            concepts.update(matches)

        # Also look for terms in bold or italic
        bold_italic_patterns = [
            r'\*\*([^*]+)\*\*',  # Bold **text**
            r'\*([^*]+)\*',      # Italic *text*
            r'__([^_]+)__'       # Bold __text__
        ]

        for pattern in bold_italic_patterns:
            matches = re.findall(pattern, content)
            concepts.update(matches)

        return list(concepts)


# Example usage and testing
if __name__ == "__main__":
    import asyncio

    async def test_content_parser():
        parser = ContentParser()

        # Create a sample MDX content for testing
        sample_content = """---
title: "Introduction to Physical AI"
author: "Textbook Team"
description: "Understanding embodied cognition in robotics"
---

# Introduction to Physical AI

Physical AI represents a paradigm shift in artificial intelligence research,
moving beyond traditional disembodied computation toward systems that learn
and adapt through physical interaction with their environment.

## Embodied Cognition

Embodied cognition is a theory in cognitive science and robotics that emphasizes
the role of an organism's body in shaping its cognitive processes. Unlike traditional
approaches that view cognition as computation occurring independently of the body,
embodied cognition suggests that cognitive processes are deeply influenced by aspects
of the physical body.

### Key Principles

The key principles of embodied cognition include:
- **Morphological Computation**: The body contributes to computation
- **Sensorimotor Coupling**: Perception and action are tightly linked
- **Environmental Interaction**: Intelligence emerges from environment interaction

## ROS 2 Fundamentals

Robot Operating System 2 (ROS 2) provides the middleware infrastructure for
robotics applications. It offers services like hardware abstraction, device
drivers, and message passing between different software components.

### Nodes and Topics

**Nodes** are the basic execution units in ROS 2, while **Topics** enable
asynchronous message passing between nodes. This architecture promotes
modular and reusable robot software development.
"""

        # Parse the sample content
        result = parser.parse_mdx_content(sample_content)

        print("Frontmatter:", result["frontmatter"])
        print(f"Word count: {result['word_count']}")
        print(f"Estimated read time: {result['read_time']} minutes")
        print(f"Number of sections: {len(result['sections'])}")

        print("\nSections:")
        for i, section in enumerate(result["sections"]):
            print(f"{i+1}. {section['title']}")
            print(f"   Content preview: {section['content'][:100]}...")
            print()

        # Test key concept extraction
        concepts = parser.extract_key_concepts(sample_content)
        print("Key concepts found:", concepts[:10])  # Show first 10 concepts

        # Test file parsing (create a temporary file for testing)
        test_file_path = "test_sample.mdx"
        async with aiofiles.open(test_file_path, 'w', encoding='utf-8') as f:
            await f.write(sample_content)

        try:
            file_result = await parser.parse_file(test_file_path)
            print(f"\nFile parsing result for {test_file_path}:")
            print(f"File name: {file_result['file_name']}")
            print(f"File type: {file_result['file_type']}")
            print(f"Word count: {file_result['word_count']}")
        finally:
            # Clean up test file
            import os
            if os.path.exists(test_file_path):
                os.remove(test_file_path)

    # Run the test
    asyncio.run(test_content_parser())