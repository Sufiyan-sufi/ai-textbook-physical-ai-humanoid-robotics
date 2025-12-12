from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.database.connection import Base

class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False)
    content = Column(Text, nullable=False)
    word_count = Column(Integer, nullable=False)
    learning_outcomes = Column(String)  # JSON string
    prerequisites = Column(String)  # JSON string
    next_chapters = Column(String)  # JSON string for UUIDs
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    version = Column(Integer, default=1)

    # Relationship
    tutorials = relationship("Tutorial", back_populates="chapter")

    def __repr__(self):
        return f"<Chapter(id={self.id}, title='{self.title}', slug='{self.slug}')>"