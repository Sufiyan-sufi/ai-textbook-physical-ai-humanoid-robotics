from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.database.connection import Base

class Tutorial(Base):
    __tablename__ = "tutorials"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(UUID(as_uuid=True), ForeignKey("chapters.id"), nullable=False)
    title = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    simulation_environment = Column(String, nullable=False)  # Enum: 'gazebo', 'isaac', 'unity'
    hardware_target = Column(String, nullable=False)  # Enum: 'jetson', 'rtx', 'simulation-only'
    estimated_duration = Column(Integer)  # in minutes
    code_examples = Column(String)  # JSON string
    learning_outcomes = Column(String)  # JSON string
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship
    chapter = relationship("Chapter", back_populates="tutorials")

    def __repr__(self):
        return f"<Tutorial(id={self.id}, title='{self.title}', chapter_id={self.chapter_id})>"