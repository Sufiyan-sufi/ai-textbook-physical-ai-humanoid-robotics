from typing import List, Dict, Any, Optional
import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import logging
from pathlib import Path
import asyncio
import aiofiles


class VectorService:
    """Service for managing vector embeddings in Qdrant for RAG retrieval"""

    def __init__(self, qdrant_url: str = "http://localhost:6333", collection_name: str = "textbook_content", qdrant_api_key: str = None, qdrant_cluster_url: str = None):
        self.qdrant_url = qdrant_url
        self.qdrant_api_key = qdrant_api_key
        self.qdrant_cluster_url = qdrant_cluster_url
        self.collection_name = collection_name
        self.model = SentenceTransformer('all-MiniLM-L6-v2')  # Lightweight model for embeddings

        # Initialize Qdrant client but don't try to connect during initialization
        # This prevents startup failures when Qdrant is not available
        try:
            from qdrant_client.http.exceptions import UnexpectedResponse

            # Use different initialization based on configuration
            if self.qdrant_cluster_url and self.qdrant_api_key:
                # Cloud Qdrant cluster configuration
                self.qdrant_client = QdrantClient(
                    url=self.qdrant_cluster_url,
                    api_key=self.qdrant_api_key,
                    prefer_grpc=True  # Use gRPC for better performance in cloud
                )
            elif self.qdrant_api_key:
                # Qdrant with API key (for self-hosted with authentication)
                self.qdrant_client = QdrantClient(
                    url=self.qdrant_url,
                    api_key=self.qdrant_api_key
                )
            else:
                # Local Qdrant (no authentication)
                self.qdrant_client = QdrantClient(url=self.qdrant_url)

            # Only try to ensure collection exists if we can connect
            self._ensure_collection_exists()
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"Could not initialize Qdrant connection: {e}. Service will attempt to connect when needed.")
            self.qdrant_client = None

    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with proper configuration"""
        try:
            # Initialize client if it's not already initialized
            if self.qdrant_client is None:
                # Use the same initialization logic as in __init__
                if self.qdrant_cluster_url and self.qdrant_api_key:
                    # Cloud Qdrant cluster configuration
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_cluster_url,
                        api_key=self.qdrant_api_key,
                        prefer_grpc=True
                    )
                elif self.qdrant_api_key:
                    # Qdrant with API key (for self-hosted with authentication)
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_url,
                        api_key=self.qdrant_api_key
                    )
                else:
                    # Local Qdrant (no authentication)
                    self.qdrant_client = QdrantClient(url=self.qdrant_url)

            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=384,  # Size of the embedding vector (all-MiniLM-L6-v2 produces 384-dim vectors)
                        distance=models.Distance.COSINE
                    )
                )
                logging.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logging.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logging.error(f"Error ensuring collection exists: {e}")
            # Don't raise the exception to allow graceful degradation
            # The service will attempt to reconnect when needed
            self.qdrant_client = None
            raise

    def create_embedding(self, text: str) -> List[float]:
        """Create embedding vector from text"""
        embedding = self.model.encode(text)
        return embedding.tolist()

    def chunk_text(self, text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
        """Split text into overlapping chunks for better retrieval"""
        words = text.split()
        chunks = []

        for i in range(0, len(words), chunk_size - overlap):
            chunk = " ".join(words[i:i + chunk_size])
            if chunk.strip():
                chunks.append(chunk)

        return chunks

    async def index_content(self, content_id: str, content: str, metadata: Dict[str, Any] = None) -> bool:
        """Index content in Qdrant with proper chunking"""
        try:
            # Initialize client if it's not already initialized
            if self.qdrant_client is None:
                # Use the same initialization logic as in __init__
                if self.qdrant_cluster_url and self.qdrant_api_key:
                    # Cloud Qdrant cluster configuration
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_cluster_url,
                        api_key=self.qdrant_api_key,
                        prefer_grpc=True
                    )
                elif self.qdrant_api_key:
                    # Qdrant with API key (for self-hosted with authentication)
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_url,
                        api_key=self.qdrant_api_key
                    )
                else:
                    # Local Qdrant (no authentication)
                    self.qdrant_client = QdrantClient(url=self.qdrant_url)

            # Chunk the content
            chunks = self.chunk_text(content)

            # Create embeddings for each chunk
            points = []
            for i, chunk in enumerate(chunks):
                embedding = self.create_embedding(chunk)

                point = models.PointStruct(
                    id=str(uuid.uuid4()),  # Use UUID instead of content_id_i format
                    vector=embedding,
                    payload={
                        "content_id": content_id,
                        "chunk_index": i,
                        "content": chunk,
                        "metadata": metadata or {}
                    }
                )
                points.append(point)

            # Upload to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logging.info(f"Indexed {len(chunks)} chunks for content {content_id}")
            return True

        except Exception as e:
            logging.error(f"Error indexing content {content_id}: {e}")
            # Reset client to None so it will reconnect on next attempt
            self.qdrant_client = None
            return False

    async def index_file(self, file_path: str, content_id: str = None, metadata: Dict[str, Any] = None) -> bool:
        """Index content from a file"""
        try:
            if content_id is None:
                content_id = str(uuid.uuid4())

            # Read file content
            async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
                content = await f.read()

            # Add file-specific metadata
            file_metadata = {
                "file_path": str(file_path),
                "file_name": Path(file_path).name,
                "content_type": "textbook_chapter"
            }

            if metadata:
                file_metadata.update(metadata)

            return await self.index_content(content_id, content, file_metadata)

        except Exception as e:
            logging.error(f"Error indexing file {file_path}: {e}")
            return False

    def search(self, query: str, limit: int = 10) -> List[Dict[str, Any]]:
        """Search for relevant content based on query"""
        try:
            # Initialize client if it's not already initialized
            if self.qdrant_client is None:
                # Use the same initialization logic as in __init__
                if self.qdrant_cluster_url and self.qdrant_api_key:
                    # Cloud Qdrant cluster configuration
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_cluster_url,
                        api_key=self.qdrant_api_key,
                        prefer_grpc=True
                    )
                elif self.qdrant_api_key:
                    # Qdrant with API key (for self-hosted with authentication)
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_url,
                        api_key=self.qdrant_api_key
                    )
                else:
                    # Local Qdrant (no authentication)
                    self.qdrant_client = QdrantClient(url=self.qdrant_url)

            query_embedding = self.create_embedding(query)

            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True
            )

            results = []
            for hit in search_results:
                results.append({
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "content_id": hit.payload.get("content_id", ""),
                    "chunk_index": hit.payload.get("chunk_index", 0),
                    "metadata": hit.payload.get("metadata", {}),
                    "score": hit.score
                })

            return results

        except Exception as e:
            logging.error(f"Error searching: {e}")
            # Reset client to None so it will reconnect on next attempt
            self.qdrant_client = None
            return []

    def delete_content(self, content_id: str) -> bool:
        """Delete all chunks associated with a content ID"""
        try:
            # Initialize client if it's not already initialized
            if self.qdrant_client is None:
                # Use the same initialization logic as in __init__
                if self.qdrant_cluster_url and self.qdrant_api_key:
                    # Cloud Qdrant cluster configuration
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_cluster_url,
                        api_key=self.qdrant_api_key,
                        prefer_grpc=True
                    )
                elif self.qdrant_api_key:
                    # Qdrant with API key (for self-hosted with authentication)
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_url,
                        api_key=self.qdrant_api_key
                    )
                else:
                    # Local Qdrant (no authentication)
                    self.qdrant_client = QdrantClient(url=self.qdrant_url)

            # Find all points with this content_id
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.content_id",
                        match=models.MatchValue(value=content_id)
                    )
                ]
            )

            points = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=search_filter,
                limit=10000  # Assuming no content will have more than 10k chunks
            )

            point_ids = [point.id for point in points[0]]

            if point_ids:
                self.qdrant_client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=point_ids
                    )
                )
                logging.info(f"Deleted {len(point_ids)} chunks for content {content_id}")

            return True

        except Exception as e:
            logging.error(f"Error deleting content {content_id}: {e}")
            # Reset client to None so it will reconnect on next attempt
            self.qdrant_client = None
            return False

    def get_content_chunks(self, content_id: str) -> List[Dict[str, Any]]:
        """Retrieve all chunks for a specific content ID"""
        try:
            # Initialize client if it's not already initialized
            if self.qdrant_client is None:
                # Use the same initialization logic as in __init__
                if self.qdrant_cluster_url and self.qdrant_api_key:
                    # Cloud Qdrant cluster configuration
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_cluster_url,
                        api_key=self.qdrant_api_key,
                        prefer_grpc=True
                    )
                elif self.qdrant_api_key:
                    # Qdrant with API key (for self-hosted with authentication)
                    self.qdrant_client = QdrantClient(
                        url=self.qdrant_url,
                        api_key=self.qdrant_api_key
                    )
                else:
                    # Local Qdrant (no authentication)
                    self.qdrant_client = QdrantClient(url=self.qdrant_url)

            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.content_id",
                        match=models.MatchValue(value=content_id)
                    )
                ]
            )

            points, _ = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=search_filter,
                limit=10000
            )

            chunks = []
            for point in points:
                chunks.append({
                    "id": point.id,
                    "content": point.payload.get("content", ""),
                    "chunk_index": point.payload.get("chunk_index", 0),
                    "metadata": point.payload.get("metadata", {})
                })

            # Sort by chunk index to maintain order
            chunks.sort(key=lambda x: x["chunk_index"])
            return chunks

        except Exception as e:
            logging.error(f"Error retrieving content chunks {content_id}: {e}")
            # Reset client to None so it will reconnect on next attempt
            self.qdrant_client = None
            return []


# Example usage and testing
if __name__ == "__main__":
    import asyncio

    async def test_vector_service():
        # Initialize the service
        service = VectorService()

        # Test content
        test_content = """
        Embodied cognition is a theory in cognitive science and robotics that emphasizes
        the role of an organism's body in shaping its cognitive processes. Unlike traditional
        approaches that view cognition as computation occurring independently of the body,
        embodied cognition suggests that cognitive processes are deeply influenced by aspects
        of the physical body, including sensorimotor systems, morphology, and environmental
        interactions.

        In robotics, embodied cognition has significant implications for how we design and
        control robots. Rather than programming robots with explicit rules for every situation,
        embodied approaches leverage the robot's physical form and interaction with the
        environment to produce intelligent behavior. This can lead to more robust and
        adaptive robotic systems that can handle real-world complexity better than purely
        symbolic AI approaches.

        The concept of morphological computation is central to embodied cognition in robotics.
        This refers to the idea that computation can be distributed between the controller
        and the body, with the physical properties of the robot contributing to intelligent
        behavior without explicit control. For example, the compliant properties of certain
        materials can naturally provide stable walking patterns without complex control algorithms.
        """

        # Index the content
        content_id = "test_chapter_1"
        success = await service.index_content(
            content_id=content_id,
            content=test_content,
            metadata={
                "chapter": "Introduction to Embodied Cognition",
                "author": "AI Textbook",
                "topic": "Physical AI"
            }
        )

        print(f"Indexing success: {success}")

        # Search for relevant content
        query = "What is embodied cognition in robotics?"
        results = service.search(query, limit=3)

        print(f"\nSearch results for '{query}':")
        for i, result in enumerate(results):
            print(f"\nResult {i+1} (Score: {result['score']:.3f}):")
            print(f"Content: {result['content'][:200]}...")

        # Test chunk retrieval
        chunks = service.get_content_chunks(content_id)
        print(f"\nRetrieved {len(chunks)} chunks for content {content_id}")

    # Run the test
    asyncio.run(test_vector_service())