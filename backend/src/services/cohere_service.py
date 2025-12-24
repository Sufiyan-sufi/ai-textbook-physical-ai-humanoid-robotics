from typing import Dict, Any, List, Optional
import logging
import os
import cohere
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class CohereService:
    """Service to handle Cohere AI operations for language understanding and generation"""

    def __init__(self):
        self.api_key = os.getenv("COHERE_API_KEY")
        if not self.api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")

        self.client = cohere.Client(api_key=self.api_key)
        self.logger = logging.getLogger(__name__)

    def generate_text(self, prompt: str, model: str = "command-r-plus",
                     max_tokens: int = 500, temperature: float = 0.7) -> Dict[str, Any]:
        """Generate text using Cohere's generative models"""
        try:
            response = self.client.generate(
                model=model,
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=temperature
            )

            if response.generations:
                return {
                    "text": response.generations[0].text,
                    "finish_reason": response.generations[0].finish_reason,
                    "prompt": prompt,
                    "model": model
                }
            else:
                return {
                    "text": "",
                    "error": "No generations returned from Cohere"
                }

        except Exception as e:
            self.logger.error(f"Error in Cohere text generation: {e}")
            return {
                "text": "",
                "error": str(e)
            }

    def embed_text(self, texts: List[str], model: str = "embed-english-v3.0",
                   input_type: str = "search_document") -> Dict[str, Any]:
        """Generate embeddings for text using Cohere's embedding models"""
        try:
            response = self.client.embed(
                texts=texts,
                model=model,
                input_type=input_type
            )

            return {
                "embeddings": response.embeddings,
                "model": model,
                "input_type": input_type
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere embedding: {e}")
            return {
                "embeddings": [],
                "error": str(e)
            }

    def classify_text(self, inputs: List[str], model: str = "large",
                     examples: List[Dict[str, str]] = None) -> Dict[str, Any]:
        """Classify text using Cohere's classification models"""
        try:
            if examples is None:
                examples = [
                    {"text": "I love this product!", "label": "positive"},
                    {"text": "This is terrible.", "label": "negative"}
                ]

            response = self.client.classify(
                model=model,
                inputs=inputs,
                examples=examples
            )

            classifications = []
            for i, classification in enumerate(response.classifications):
                classifications.append({
                    "input": inputs[i],
                    "prediction": classification.prediction,
                    "confidence": classification.confidence,
                    "labels": {label: score for label, score in classification.labels.items()}
                })

            return {
                "classifications": classifications,
                "model": model
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere classification: {e}")
            return {
                "classifications": [],
                "error": str(e)
            }

    def summarize_text(self, text: str, model: str = "summarize-xlarge",
                      length: str = "medium", format: str = "paragraph",
                      extractiveness: str = "medium", temperature: float = 0.3) -> Dict[str, Any]:
        """Summarize text using Cohere's summarization models"""
        try:
            response = self.client.summarize(
                text=text,
                model=model,
                length=length,
                format=format,
                extractiveness=extractiveness,
                temperature=temperature
            )

            return {
                "summary": response.summary,
                "model": model,
                "length": length,
                "format": format
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere summarization: {e}")
            return {
                "summary": "",
                "error": str(e)
            }

    def rank_documents(self, query: str, documents: List[str],
                      model: str = "rerank-english-v2.0", top_n: Optional[int] = None) -> Dict[str, Any]:
        """Rank documents based on relevance to a query using Cohere's reranking"""
        try:
            response = self.client.rerank(
                model=model,
                query=query,
                documents=documents,
                top_n=top_n
            )

            ranked_results = []
            for i, result in enumerate(response.results):
                ranked_results.append({
                    "index": result.index,
                    "document": documents[result.index],
                    "relevance_score": result.relevance_score,
                    "rank": i + 1
                })

            return {
                "ranked_results": ranked_results,
                "model": model,
                "query": query
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere ranking: {e}")
            return {
                "ranked_results": [],
                "error": str(e)
            }

    def chat_with_cohere(self, message: str, conversation_id: Optional[str] = None,
                        model: str = "command-r-plus",
                        temperature: float = 0.7) -> Dict[str, Any]:
        """Chat with Cohere's conversational models"""
        try:
            # Prepare the chat request
            chat_request = {
                "message": message,
                "model": model,
                "temperature": temperature
            }

            if conversation_id:
                chat_request["conversation_id"] = conversation_id

            response = self.client.chat(**chat_request)

            return {
                "response": response.text,
                "conversation_id": response.conversation_id,
                "model": model,
                "finish_reason": response.finish_reason
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere chat: {e}")
            return {
                "response": "",
                "error": str(e)
            }

    def detect_language(self, texts: List[str]) -> Dict[str, Any]:
        """Detect the language of given texts"""
        try:
            response = self.client.detect_language(texts=texts)

            languages = []
            for i, language in enumerate(response.results):
                languages.append({
                    "text": texts[i],
                    "language_code": language.language_code,
                    "language_name": language.language_name,
                    "confidence": language.confidence
                })

            return {
                "languages": languages
            }

        except Exception as e:
            self.logger.error(f"Error in Cohere language detection: {e}")
            return {
                "languages": [],
                "error": str(e)
            }