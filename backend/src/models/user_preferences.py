from sqlalchemy import Column, String, Text, DateTime
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.ext.mutable import MutableDict
from datetime import datetime
from src.database.connection import Base


class UserPreferences(Base):
    """Database model for user preferences and personalization settings"""

    __tablename__ = "user_preferences"

    # Primary key
    id = Column(String, primary_key=True, index=True)

    # Foreign key to user
    user_id = Column(String, unique=True, index=True, nullable=False)

    # Learning preferences
    learning_level = Column(String, default="beginner")  # beginner, intermediate, advanced
    learning_style = Column(String, default="visual")   # visual, auditory, kinesthetic, reading/writing

    # Interest and topic preferences
    interests = Column(Text)  # JSON string of user interests
    preferred_topics = Column(Text)  # JSON string of preferred topics

    # Content preferences
    preferred_language = Column(String, default="en")  # Default language
    text_size_preference = Column(String, default="medium")  # small, medium, large
    theme_preference = Column(String, default="light")  # light, dark, high-contrast

    # Accessibility settings
    accessibility_settings = Column(Text)  # JSON string of accessibility preferences

    # Learning progress
    last_accessed_chapters = Column(Text)  # JSON string of chapter IDs
    completed_chapters = Column(Text)  # JSON string of completed chapter IDs

    # Customization settings
    customizations = Column(Text)  # JSON string of custom settings

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<UserPreferences(user_id={self.user_id}, learning_level={self.learning_level})>"

    def to_dict(self):
        """Convert the model instance to a dictionary"""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "learning_level": self.learning_level,
            "learning_style": self.learning_style,
            "interests": self.interests,
            "preferred_topics": self.preferred_topics,
            "preferred_language": self.preferred_language,
            "text_size_preference": self.text_size_preference,
            "theme_preference": self.theme_preference,
            "accessibility_settings": self.accessibility_settings,
            "last_accessed_chapters": self.last_accessed_chapters,
            "completed_chapters": self.completed_chapters,
            "customizations": self.customizations,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }