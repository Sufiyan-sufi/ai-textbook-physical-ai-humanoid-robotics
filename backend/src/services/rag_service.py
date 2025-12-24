from typing import Dict, Any, List, Optional
import logging
from src.services.vector_service import VectorService
from src.utils.content_parser import ContentParser
from src.services.cohere_service import CohereService


class RAGService:
    """Retrieval Augmented Generation service for textbook content retrieval"""

    def __init__(self, qdrant_url: str = None, qdrant_api_key: str = None, qdrant_cluster_url: str = None):
        self.qdrant_url = qdrant_url or "http://localhost:6333"
        self.qdrant_api_key = qdrant_api_key
        self.qdrant_cluster_url = qdrant_cluster_url
        self.content_parser = ContentParser()
        self.logger = logging.getLogger(__name__)

        # Initialize VectorService - don't fail if Qdrant is not available
        try:
            self.vector_service = VectorService(
                qdrant_url=self.qdrant_url,
                qdrant_api_key=self.qdrant_api_key,
                qdrant_cluster_url=self.qdrant_cluster_url
            )
            self.qdrant_available = True
            self.logger.info("Vector service initialized successfully")
        except Exception as e:
            self.vector_service = None
            self.qdrant_available = False
            self.logger.warning(f"Vector service not available: {e}")

        # Initialize Cohere service for advanced reranking and NLP tasks
        try:
            self.cohere_service = CohereService()
            self.cohere_available = True
            self.logger.info("Cohere service initialized successfully")
        except Exception as e:
            self.cohere_service = None
            self.cohere_available = False
            self.logger.warning(f"Cohere service not available: {e}")

    def query(self, query_text: str, context: Optional[Dict[str, Any]] = None, top_k: int = 5) -> Dict[str, Any]:
        """
        Query the textbook content using RAG

        Args:
            query_text: The user's query
            context: Additional context for the query
            top_k: Number of top results to return

        Returns:
            Dictionary containing results and metadata
        """
        try:
            # Check if vector service is available
            if not self.qdrant_available or self.vector_service is None:
                self.logger.warning("Vector service not available, returning empty results")
                return {
                    "query": query_text,
                    "answer": "I'm sorry, but I couldn't access the textbook content to answer your question. Please try again later.",
                    "results": [],
                    "retrieved_context": "",
                    "total_results": 0,
                    "search_metadata": {
                        "top_k": top_k,
                        "vector_database": "qdrant",
                        "status": "unavailable"
                    }
                }

            # Search for relevant content in the vector database
            search_results = self.vector_service.search(query_text, limit=top_k)

            # Format the results for the AI model
            formatted_results = []
            for result in search_results:
                formatted_result = {
                    "content": result["content"],
                    "content_id": result["content_id"],
                    "score": result["score"],
                    "metadata": result["metadata"],
                    "chunk_index": result.get("chunk_index", 0)
                }
                formatted_results.append(formatted_result)

            # Prepare context for the AI model
            retrieved_context = self._format_context_for_llm(formatted_results)

            # Generate a specific answer using the retrieved context
            answer = self._generate_answer(query_text, retrieved_context)

            response = {
                "query": query_text,
                "answer": answer,
                "results": formatted_results,
                "retrieved_context": retrieved_context,
                "total_results": len(formatted_results),
                "search_metadata": {
                    "top_k": top_k,
                    "query_embedding_used": True,
                    "vector_database": "qdrant"
                }
            }

            self.logger.info(f"RAG query completed for: '{query_text[:50]}...' - Found {len(formatted_results)} results")
            return response

        except Exception as e:
            self.logger.error(f"Error in RAG query: {e}")
            return {
                "query": query_text,
                "answer": "I encountered an error while processing your request. Please try again.",
                "results": [],
                "retrieved_context": "",
                "total_results": 0,
                "error": str(e),
                "search_metadata": {
                    "top_k": top_k,
                    "error": str(e)
                }
            }

    def _format_context_for_llm(self, results: List[Dict[str, Any]]) -> str:
        """Format retrieved results into a context string for the LLM"""
        if not results:
            return "No relevant content found in the textbook."

        formatted_context = []
        for i, result in enumerate(results):
            content = result["content"]
            metadata = result["metadata"]
            score = result["score"]

            # Create a formatted context entry
            context_entry = f"""
[Relevant Content {i+1} (Relevance Score: {score:.3f})]
Source: {metadata.get('file_name', 'Unknown')}
Chapter: {metadata.get('chapter_title', 'Unknown')}
Content: {content}
[End of Content {i+1}]
"""
            formatted_context.append(context_entry.strip())

        return "\\n\\n".join(formatted_context)

    def _generate_answer(self, query_text: str, retrieved_context: str) -> str:
        """Generate a specific answer based on the query and retrieved context"""
        if not retrieved_context or "No relevant content found" in retrieved_context:
            return "I couldn't find relevant content in the textbook to answer your question. Please try rephrasing your question or ask about a different topic."

        try:
            from openai import OpenAI
            import os

            # Initialize OpenAI client
            client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

            # Create a prompt to generate a specific answer
            prompt = f"""
Based on the following textbook content, please answer the user's question concisely and accurately.
Only use information from the provided context. If the context doesn't contain enough information to answer the question, say so.

Context:
{retrieved_context}

Question: {query_text}

Answer:"""

            response = client.chat.completions.create(
                model="gpt-4-turbo",  # Using a capable model for textbook content
                messages=[
                    {
                        "role": "system",
                        "content": "You are an AI tutor helping students with Physical AI & Humanoid Robotics. Provide clear, accurate, and concise answers based on the provided textbook content. If the content doesn't answer the question, acknowledge that and suggest rephrasing or asking a different question."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                max_tokens=500,  # Limit response length for conciseness
                temperature=0.3,  # Lower temperature for more factual responses
                top_p=0.9
            )

            answer = response.choices[0].message.content.strip()

            # Ensure the answer is relevant and concise
            if not answer or len(answer.strip()) < 10:
                return "I found relevant content in the textbook, but I'm unable to generate a specific answer. Please try rephrasing your question."

            return answer

        except Exception as e:
            self.logger.error(f"Error generating answer with OpenAI: {e}")
            # Fallback: return a summary of the context
            if retrieved_context and len(retrieved_context) > 500:
                return retrieved_context[:500] + "... (content truncated for brevity)"
            else:
                return retrieved_context if retrieved_context else "I found some content but couldn't generate a specific answer. Please try rephrasing your question."

    def query_with_reranking(self, query_text: str, context: Optional[Dict[str, Any]] = None,
                           top_k: int = 5, rerank_limit: int = 10) -> Dict[str, Any]:
        """
        Query with additional reranking for better results
        """
        try:
            # Check if vector service is available
            if not self.qdrant_available or self.vector_service is None:
                self.logger.warning("Vector service not available, returning empty results")
                return {
                    "query": query_text,
                    "results": [],
                    "retrieved_context": "",
                    "total_results": 0,
                    "search_metadata": {
                        "top_k": top_k,
                        "rerank_limit": rerank_limit,
                        "vector_database": "qdrant",
                        "status": "unavailable"
                    }
                }

            # Get more results than needed for reranking
            initial_results = self.vector_service.search(query_text, limit=rerank_limit)

            # Simple reranking based on content relevance and metadata
            reranked_results = self._rerank_results(initial_results, query_text)

            # Take top_k from reranked results
            final_results = reranked_results[:top_k]

            # Format the results
            formatted_results = []
            for result in final_results:
                formatted_result = {
                    "content": result["content"],
                    "content_id": result["content_id"],
                    "score": result["score"],
                    "metadata": result["metadata"],
                    "chunk_index": result.get("chunk_index", 0),
                    "reranked_score": result.get("reranked_score", result["score"])
                }
                formatted_results.append(formatted_result)

            retrieved_context = self._format_context_for_llm(formatted_results)

            # Generate a specific answer using the retrieved context
            answer = self._generate_answer(query_text, retrieved_context)

            response = {
                "query": query_text,
                "answer": answer,
                "results": formatted_results,
                "retrieved_context": retrieved_context,
                "total_results": len(formatted_results),
                "search_metadata": {
                    "top_k": top_k,
                    "rerank_limit": rerank_limit,
                    "query_embedding_used": True,
                    "vector_database": "qdrant",
                    "reranking_applied": True
                }
            }

            self.logger.info(f"RAG query with reranking completed for: '{query_text[:50]}...' - Found {len(formatted_results)} results")
            return response

        except Exception as e:
            self.logger.error(f"Error in RAG query with reranking: {e}")
            return self.query(query_text, context, top_k)  # Fallback to regular query

    def _rerank_results(self, results: List[Dict[str, Any]], query_text: str) -> List[Dict[str, Any]]:
        """Simple reranking algorithm to improve result quality"""
        # If Cohere is available, use its advanced reranking
        if self.cohere_available:
            return self._rerank_with_cohere(results, query_text)
        else:
            # Fallback to basic reranking
            return self._rerank_with_basic_algorithm(results, query_text)

    def _rerank_with_cohere(self, results: List[Dict[str, Any]], query_text: str) -> List[Dict[str, Any]]:
        """Use Cohere's advanced reranking for better result quality"""
        try:
            # Extract content from results for Cohere reranking
            documents = [result["content"] for result in results]

            # Use Cohere's rerank functionality
            cohere_response = self.cohere_service.rank_documents(
                query=query_text,
                documents=documents,
                top_n=len(documents)
            )

            if "error" in cohere_response:
                self.logger.warning(f"Cohere reranking failed: {cohere_response['error']}")
                # Fall back to basic algorithm
                return self._rerank_with_basic_algorithm(results, query_text)

            # Reorder results based on Cohere's ranking
            reranked = []
            for result in cohere_response["ranked_results"]:
                original_result = results[result["index"]]
                reranked_result = original_result.copy()
                reranked_result["reranked_score"] = result["relevance_score"]
                reranked.append(reranked_result)

            return reranked

        except Exception as e:
            self.logger.error(f"Error in Cohere reranking: {e}")
            # Fall back to basic algorithm
            return self._rerank_with_basic_algorithm(results, query_text)

    def _rerank_with_basic_algorithm(self, results: List[Dict[str, Any]], query_text: str) -> List[Dict[str, Any]]:
        """Simple reranking algorithm to improve result quality"""
        reranked = []

        for result in results:
            # Calculate additional relevance factors
            content = result["content"].lower()
            query_terms = query_text.lower().split()

            # Count term matches
            term_matches = sum(1 for term in query_terms if term in content)

            # Calculate term density score
            total_terms = len(content.split())
            term_density = term_matches / max(total_terms, 1)

            # Combine original score with new factors
            reranked_score = result["score"] * 0.7 + (term_density * 0.3)

            reranked_result = result.copy()
            reranked_result["reranked_score"] = reranked_score
            reranked.append(reranked_result)

        # Sort by reranked score
        reranked.sort(key=lambda x: x["reranked_score"], reverse=True)
        return reranked

    def get_content_by_id(self, content_id: str) -> Dict[str, Any]:
        """Retrieve all chunks for a specific content ID"""
        try:
            # Check if vector service is available
            if not self.qdrant_available or self.vector_service is None:
                self.logger.warning("Vector service not available, returning empty results")
                return {
                    "content_id": content_id,
                    "chunks": [],
                    "content": "",
                    "found": False,
                    "search_metadata": {
                        "status": "unavailable"
                    }
                }

            chunks = self.vector_service.get_content_chunks(content_id)

            if not chunks:
                return {
                    "content_id": content_id,
                    "chunks": [],
                    "content": "",
                    "found": False
                }

            # Combine all chunks to reconstruct the full content
            sorted_chunks = sorted(chunks, key=lambda x: x["chunk_index"])
            full_content = " ".join([chunk["content"] for chunk in sorted_chunks])

            return {
                "content_id": content_id,
                "chunks": sorted_chunks,
                "content": full_content,
                "found": True,
                "total_chunks": len(sorted_chunks)
            }

        except Exception as e:
            self.logger.error(f"Error retrieving content by ID {content_id}: {e}")
            return {
                "content_id": content_id,
                "chunks": [],
                "content": "",
                "found": False,
                "error": str(e)
            }

    def add_content(self, content_id: str, content: str, metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Add new content to the vector database"""
        try:
            # Check if vector service is available
            if not self.qdrant_available or self.vector_service is None:
                self.logger.warning("Vector service not available, cannot add content")
                return False

            success = self.vector_service.index_content(content_id, content, metadata or {})
            if success:
                self.logger.info(f"Successfully added content to RAG: {content_id}")
            else:
                self.logger.error(f"Failed to add content to RAG: {content_id}")
            return success
        except Exception as e:
            self.logger.error(f"Error adding content {content_id} to RAG: {e}")
            return False

    def delete_content(self, content_id: str) -> bool:
        """Remove content from the vector database"""
        try:
            # Check if vector service is available
            if not self.qdrant_available or self.vector_service is None:
                self.logger.warning("Vector service not available, cannot delete content")
                return False

            success = self.vector_service.delete_content(content_id)
            if success:
                self.logger.info(f"Successfully removed content from RAG: {content_id}")
            else:
                self.logger.error(f"Failed to remove content from RAG: {content_id}")
            return success
        except Exception as e:
            self.logger.error(f"Error removing content {content_id} from RAG: {e}")
            return False