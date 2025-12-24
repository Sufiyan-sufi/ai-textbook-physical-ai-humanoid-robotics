from typing import List, Dict, Any
import asyncio
import logging
from pathlib import Path
import sys
import os

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.vector_service import VectorService
from src.utils.content_parser import ContentParser


class VectorInitializer:
    """Initialize vector embeddings for the textbook content"""

    def __init__(self, qdrant_url: str = "http://localhost:6333", qdrant_api_key: str = None, qdrant_cluster_url: str = None, textbook_path: str = None):
        self.vector_service = VectorService(
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            qdrant_cluster_url=qdrant_cluster_url
        )
        self.content_parser = ContentParser()
        self.textbook_path = textbook_path or "my-website/content/chapters"
        self.logger = logging.getLogger(__name__)

    async def initialize_vectors(self) -> bool:
        """Initialize vectors by processing all textbook content"""
        try:
            self.logger.info(f"Starting vector initialization for content in: {self.textbook_path}")

            # Parse all textbook content
            textbook_files = await self.content_parser.parse_directory(
                self.textbook_path,
                file_extensions=['.md', '.mdx']
            )

            self.logger.info(f"Found {len(textbook_files)} content files to process")

            # Process each file and index its content
            processed_count = 0
            for file_data in textbook_files:
                try:
                    file_path = file_data["file_path"]
                    content_id = f"chapter_{Path(file_path).stem}_{hash(file_path) % 10000}"

                    # Create metadata for the content
                    metadata = {
                        "file_path": file_path,
                        "file_name": file_data["file_name"],
                        "chapter_title": file_data.get("frontmatter", {}).get("title", ""),
                        "author": file_data.get("frontmatter", {}).get("author", ""),
                        "description": file_data.get("frontmatter", {}).get("description", ""),
                        "word_count": file_data.get("word_count", 0),
                        "read_time": file_data.get("read_time", 0),
                        "content_type": "textbook_chapter"
                    }

                    # Index the content
                    success = await self.vector_service.index_content(
                        content_id=content_id,
                        content=file_data["content"],
                        metadata=metadata
                    )

                    if success:
                        processed_count += 1
                        self.logger.info(f"Successfully indexed: {file_path}")
                    else:
                        self.logger.error(f"Failed to index: {file_path}")

                except Exception as e:
                    self.logger.error(f"Error processing file {file_data.get('file_path', 'unknown')}: {e}")

            self.logger.info(f"Successfully processed {processed_count} out of {len(textbook_files)} files")
            return processed_count > 0

        except Exception as e:
            self.logger.error(f"Error initializing vectors: {e}")
            return False

    async def reinitialize_content(self, content_id: str, new_content: str, metadata: Dict[str, Any] = None) -> bool:
        """Reinitialize a specific content item"""
        try:
            # Delete existing content
            self.vector_service.delete_content(content_id)

            # Index the new content
            success = await self.vector_service.index_content(
                content_id=content_id,
                content=new_content,
                metadata=metadata or {}
            )

            if success:
                self.logger.info(f"Successfully reinitialized content: {content_id}")
            else:
                self.logger.error(f"Failed to reinitialize content: {content_id}")

            return success

        except Exception as e:
            self.logger.error(f"Error reinitializing content {content_id}: {e}")
            return False

    async def test_retrieval(self) -> bool:
        """Test retrieval functionality"""
        try:
            self.logger.info("Testing retrieval functionality...")

            # Test search with a sample query
            test_query = "What is embodied cognition in robotics?"
            results = self.vector_service.search(test_query, limit=5)

            if results:
                self.logger.info(f"Retrieved {len(results)} results for query: '{test_query[:50]}...'")
                for i, result in enumerate(results[:3]):  # Show first 3 results
                    print(f"\nResult {i+1} (Score: {result['score']:.3f}):")
                    print(f"Content preview: {result['content'][:200]}...")
                    print(f"Metadata: {result['metadata']}")
            else:
                self.logger.info("No results found for test query (this is expected if no content is indexed yet)")

            return True

        except Exception as e:
            self.logger.error(f"Error testing retrieval: {e}")
            return False

    async def get_content_stats(self) -> Dict[str, Any]:
        """Get statistics about indexed content"""
        try:
            # This would require Qdrant collection info in a real implementation
            # For now, return placeholder stats
            stats = {
                "total_content_items": 0,
                "total_chunks": 0,
                "average_chunk_size": 0,
                "indexed_files": []
            }

            # In a real implementation, we would query Qdrant for this information
            # This is just a placeholder
            return stats

        except Exception as e:
            self.logger.error(f"Error getting content stats: {e}")
            return {}


async def main():
    """Main function to run the vector initialization"""
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Initialize the vector initializer
    initializer = VectorInitializer(
        qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
        qdrant_api_key=os.getenv("QDRANT_API_KEY"),
        qdrant_cluster_url=os.getenv("QDRANT_CLUSTER_URL"),
        textbook_path=os.getenv("TEXTBOOK_PATH", "my-website/content/chapters")
    )

    print("Starting vector initialization process...")
    print(f"Textbook path: {initializer.textbook_path}")

    # Initialize vectors
    success = await initializer.initialize_vectors()

    if success:
        print("Vector initialization completed successfully!")
    else:
        print("Vector initialization failed!")
        return False

    # Test retrieval
    print("\nTesting retrieval functionality...")
    test_success = await initializer.test_retrieval()

    if test_success:
        print("Retrieval test completed successfully!")
    else:
        print("Retrieval test failed!")

    # Get and display stats
    stats = await initializer.get_content_stats()
    print(f"\nContent statistics: {stats}")

    return success and test_success


if __name__ == "__main__":
    # Run the initialization
    success = asyncio.run(main())
    if success:
        print("\nVector initialization process completed successfully!")
        exit(0)
    else:
        print("\nVector initialization process failed!")
        exit(1)