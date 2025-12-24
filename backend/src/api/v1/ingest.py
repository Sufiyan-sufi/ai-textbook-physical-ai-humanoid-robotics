from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import asyncio
import logging
import os
from pathlib import Path

from src.services.vector_service import VectorService
from src.services.rag_service import RAGService
from src.utils.content_parser import ContentParser
from src.database.connection import get_db, AsyncSessionLocal
from src.models.chapter import Chapter

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services with environment variables
vector_service = VectorService(
    qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    qdrant_api_key=os.getenv("QDRANT_API_KEY"),
    qdrant_cluster_url=os.getenv("QDRANT_CLUSTER_URL")
)
rag_service = RAGService(
    qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    qdrant_api_key=os.getenv("QDRANT_API_KEY"),
    qdrant_cluster_url=os.getenv("QDRANT_CLUSTER_URL")
)
content_parser = ContentParser()

class IngestRequest(BaseModel):
    source_type: str  # 'file', 'directory', 'url', 'text'
    source_path: str  # Path to file/directory or URL
    content_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class IngestResponse(BaseModel):
    success: bool
    message: str
    processed_items: int
    content_ids: List[str]

class DeleteRequest(BaseModel):
    content_id: str

class DeleteResponse(BaseModel):
    success: bool
    message: str

class StatusResponse(BaseModel):
    total_documents: int
    vector_db_status: str
    qdrant_cluster_id: Optional[str]

@router.post("/ingest", response_model=IngestResponse)
async def ingest_content(request: IngestRequest):
    """Ingest content into the vector database (Qdrant) for semantic search"""
    try:
        processed_count = 0
        content_ids = []

        if request.source_type == "file":
            # Process single file
            content_id = request.content_id or f"file_{hash(request.source_path) % 10000}"
            async with AsyncSessionLocal() as session:
                # Check if chapter exists in database
                result = await session.execute(
                    Chapter.__table__.select().where(Chapter.slug == Path(request.source_path).stem)
                )
                chapter = result.fetchone()

                if chapter:
                    metadata = {
                        "chapter_id": chapter.id,
                        "title": chapter.title,
                        "source_file": request.source_path,
                        "content_type": "textbook_chapter",
                        **(request.metadata or {})
                    }
                else:
                    metadata = {
                        "source_file": request.source_path,
                        "content_type": "textbook_content",
                        **(request.metadata or {})
                    }

            success = await vector_service.index_file(
                file_path=request.source_path,
                content_id=content_id,
                metadata=metadata
            )

            if success:
                processed_count = 1
                content_ids = [content_id]
                logger.info(f"Successfully ingested file: {request.source_path}")
            else:
                raise HTTPException(status_code=500, detail=f"Failed to ingest file: {request.source_path}")

        elif request.source_type == "directory":
            # Process directory of files
            textbook_files = await content_parser.parse_directory(
                request.source_path,
                file_extensions=['.md', '.mdx', '.txt', '.pdf']
            )

            for file_data in textbook_files:
                try:
                    file_path = file_data["file_path"]
                    content_id = request.content_id or f"chapter_{Path(file_path).stem}_{hash(file_path) % 10000}"

                    metadata = {
                        "file_path": file_path,
                        "file_name": file_data["file_name"],
                        "chapter_title": file_data.get("frontmatter", {}).get("title", ""),
                        "author": file_data.get("frontmatter", {}).get("author", ""),
                        "description": file_data.get("frontmatter", {}).get("description", ""),
                        "word_count": file_data.get("word_count", 0),
                        "read_time": file_data.get("read_time", 0),
                        "content_type": "textbook_chapter",
                        **(request.metadata or {})
                    }

                    success = await vector_service.index_content(
                        content_id=content_id,
                        content=file_data["content"],
                        metadata=metadata
                    )

                    if success:
                        processed_count += 1
                        content_ids.append(content_id)
                        logger.info(f"Successfully ingested: {file_path}")
                    else:
                        logger.error(f"Failed to ingest: {file_path}")

                except Exception as e:
                    logger.error(f"Error processing file {file_data.get('file_path', 'unknown')}: {e}")

        elif request.source_type == "text":
            # Process raw text content
            content_id = request.content_id or f"text_{hash(request.source_path) % 10000}"
            metadata = {
                "source_type": "raw_text",
                "content_type": "textbook_content",
                **(request.metadata or {})
            }

            success = await vector_service.index_content(
                content_id=content_id,
                content=request.source_path,  # In this case, source_path contains the actual text
                metadata=metadata
            )

            if success:
                processed_count = 1
                content_ids = [content_id]
                logger.info(f"Successfully ingested raw text content")
            else:
                raise HTTPException(status_code=500, detail="Failed to ingest raw text content")

        else:
            raise HTTPException(status_code=400, detail=f"Unsupported source type: {request.source_type}")

        return IngestResponse(
            success=True,
            message=f"Successfully processed {processed_count} items",
            processed_items=processed_count,
            content_ids=content_ids
        )

    except Exception as e:
        logger.error(f"Error in ingest_content: {e}")
        raise HTTPException(status_code=500, detail=f"Error ingesting content: {str(e)}")

@router.delete("/ingest/{content_id}", response_model=DeleteResponse)
async def delete_content(content_id: str):
    """Delete content from the vector database"""
    try:
        success = vector_service.delete_content(content_id)

        if success:
            logger.info(f"Successfully deleted content: {content_id}")
            return DeleteResponse(
                success=True,
                message=f"Successfully deleted content: {content_id}"
            )
        else:
            logger.error(f"Failed to delete content: {content_id}")
            raise HTTPException(status_code=500, detail=f"Failed to delete content: {content_id}")

    except Exception as e:
        logger.error(f"Error deleting content {content_id}: {e}")
        raise HTTPException(status_code=500, detail=f"Error deleting content: {str(e)}")

@router.delete("/ingest", response_model=DeleteResponse)
async def delete_content_by_request(request: DeleteRequest):
    """Delete content from the vector database using request body"""
    try:
        success = vector_service.delete_content(request.content_id)

        if success:
            logger.info(f"Successfully deleted content: {request.content_id}")
            return DeleteResponse(
                success=True,
                message=f"Successfully deleted content: {request.content_id}"
            )
        else:
            logger.error(f"Failed to delete content: {request.content_id}")
            raise HTTPException(status_code=500, detail=f"Failed to delete content: {request.content_id}")

    except Exception as e:
        logger.error(f"Error deleting content {request.content_id}: {e}")
        raise HTTPException(status_code=500, detail=f"Error deleting content: {str(e)}")

@router.get("/status", response_model=StatusResponse)
async def get_ingestion_status():
    """Get the status of the vector database"""
    try:
        # This would typically query Qdrant for collection stats
        # For now, we'll return a placeholder response
        qdrant_cluster_id = os.getenv("QDRANT_CLUSTER_ID")

        # In a real implementation, we would get actual stats from Qdrant
        # For now, return a placeholder
        return StatusResponse(
            total_documents=0,  # This would come from Qdrant
            vector_db_status="connected",
            qdrant_cluster_id=qdrant_cluster_id
        )
    except Exception as e:
        logger.error(f"Error getting ingestion status: {e}")
        raise HTTPException(status_code=500, detail=f"Error getting status: {str(e)}")

@router.post("/reindex", response_model=IngestResponse)
async def reindex_content(request: IngestRequest):
    """Reindex content (delete and re-ingest)"""
    try:
        # First, try to delete existing content if content_id is provided
        if request.content_id:
            delete_success = vector_service.delete_content(request.content_id)
            if not delete_success:
                logger.warning(f"Content {request.content_id} may not have existed for deletion")

        # Then ingest the content again
        ingest_request = IngestRequest(
            source_type=request.source_type,
            source_path=request.source_path,
            content_id=request.content_id,
            metadata=request.metadata
        )

        return await ingest_content(ingest_request)

    except Exception as e:
        logger.error(f"Error reindexing content: {e}")
        raise HTTPException(status_code=500, detail=f"Error reindexing content: {str(e)}")

@router.post("/batch-ingest", response_model=IngestResponse)
async def batch_ingest_content(requests: List[IngestRequest]):
    """Batch ingest multiple content items"""
    try:
        total_processed = 0
        all_content_ids = []

        for req in requests:
            try:
                # Process each request individually
                result = await ingest_content(req)
                total_processed += result.processed_items
                all_content_ids.extend(result.content_ids)
            except Exception as e:
                logger.error(f"Error processing batch request {req.source_path}: {e}")
                # Continue with other requests even if one fails

        return IngestResponse(
            success=True,
            message=f"Successfully processed {total_processed} items across {len(requests)} requests",
            processed_items=total_processed,
            content_ids=all_content_ids
        )

    except Exception as e:
        logger.error(f"Error in batch ingestion: {e}")
        raise HTTPException(status_code=500, detail=f"Error in batch ingestion: {str(e)}")