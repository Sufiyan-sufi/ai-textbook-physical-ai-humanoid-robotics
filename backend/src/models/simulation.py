from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from src.database.connection import Base

class SimulationSession(Base):
    __tablename__ = "simulation_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), nullable=False)  # Could be a foreign key if User table exists
    chapter_id = Column(UUID(as_uuid=True), nullable=False)  # Could be a foreign key if Chapter table exists
    tutorial_id = Column(UUID(as_uuid=True), nullable=True)  # Could be a foreign key if Tutorial table exists
    environment = Column(String, nullable=False)  # Enum: 'gazebo', 'isaac', 'unity'
    status = Column(String, default='created')  # Enum: 'created', 'running', 'completed', 'failed', 'terminated'
    parameters = Column(String)  # JSON string for simulation configuration
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    started_at = Column(DateTime(timezone=True))
    completed_at = Column(DateTime(timezone=True))

    def __repr__(self):
        return f"<SimulationSession(id={self.id}, user_id={self.user_id}, status='{self.status}')>"