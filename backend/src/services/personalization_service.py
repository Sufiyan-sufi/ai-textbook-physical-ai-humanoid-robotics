from typing import Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import selectinload
from src.database.connection import get_db
from src.models.user import User
from src.models.user_preferences import UserPreferences
import json
import logging


class PersonalizationService:
    """Service for managing user personalization preferences"""

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    async def get_user_preferences(self, user_id: str, db: AsyncSession) -> Optional[Dict[str, Any]]:
        """Get user preferences from database"""
        try:
            result = await db.execute(
                select(UserPreferences).where(UserPreferences.user_id == user_id)
            )
            preferences = result.scalars().first()

            if preferences:
                return {
                    "user_id": preferences.user_id,
                    "learning_level": preferences.learning_level,
                    "interests": json.loads(preferences.interests) if preferences.interests else [],
                    "preferred_topics": json.loads(preferences.preferred_topics) if preferences.preferred_topics else [],
                    "learning_style": preferences.learning_style,
                    "accessibility_settings": json.loads(preferences.accessibility_settings) if preferences.accessibility_settings else {},
                    "last_accessed_chapters": json.loads(preferences.last_accessed_chapters) if preferences.last_accessed_chapters else [],
                    "completed_chapters": json.loads(preferences.completed_chapters) if preferences.completed_chapters else [],
                    "customizations": json.loads(preferences.customizations) if preferences.customizations else {}
                }
            else:
                # Return default preferences if none exist
                return {
                    "user_id": user_id,
                    "learning_level": "beginner",
                    "interests": [],
                    "preferred_topics": [],
                    "learning_style": "visual",
                    "accessibility_settings": {},
                    "last_accessed_chapters": [],
                    "completed_chapters": [],
                    "customizations": {}
                }

        except Exception as e:
            self.logger.error(f"Error getting user preferences for {user_id}: {e}")
            return None

    async def update_user_preferences(self, user_id: str, preferences_data: Dict[str, Any], db: AsyncSession) -> bool:
        """Update user preferences in database"""
        try:
            # Get existing preferences or create new ones
            result = await db.execute(
                select(UserPreferences).where(UserPreferences.user_id == user_id)
            )
            existing_prefs = result.scalars().first()

            if existing_prefs:
                # Update existing preferences
                for key, value in preferences_data.items():
                    if hasattr(UserPreferences, key):
                        if isinstance(value, (dict, list)):
                            setattr(existing_prefs, key, json.dumps(value))
                        else:
                            setattr(existing_prefs, key, value)
            else:
                # Create new preferences
                prefs_dict = {"user_id": user_id}
                for key, value in preferences_data.items():
                    if hasattr(UserPreferences, key):
                        if isinstance(value, (dict, list)):
                            prefs_dict[key] = json.dumps(value)
                        else:
                            prefs_dict[key] = value

                new_prefs = UserPreferences(**prefs_dict)
                db.add(new_prefs)

            await db.commit()
            return True

        except Exception as e:
            self.logger.error(f"Error updating user preferences for {user_id}: {e}")
            await db.rollback()
            return False

    async def personalize_content(self, user_id: str, content: str, db: AsyncSession,
                                 context: Optional[Dict[str, Any]] = None) -> str:
        """Personalize content based on user preferences"""
        try:
            user_prefs = await self.get_user_preferences(user_id, db)
            if not user_prefs:
                return content  # Return original content if no preferences found

            personalized_content = content

            # Adjust content based on learning level
            if user_prefs.get("learning_level") == "beginner":
                # Add more explanations and examples for beginners
                personalized_content = self._add_beginner_explanations(personalized_content)
            elif user_prefs.get("learning_level") == "advanced":
                # Add more technical depth for advanced users
                personalized_content = self._add_advanced_details(personalized_content)

            # Adjust based on learning style
            learning_style = user_prefs.get("learning_style", "visual")
            if learning_style == "visual":
                # Add visual cues or emphasize visual content
                personalized_content = self._enhance_visual_elements(personalized_content)
            elif learning_style == "kinesthetic":
                # Add more hands-on examples
                personalized_content = self._add_hands_on_elements(personalized_content)

            # Adjust based on user interests
            interests = user_prefs.get("interests", [])
            if interests and context:
                # Tailor examples based on user interests
                personalized_content = self._tailor_examples_by_interests(
                    personalized_content, interests, context
                )

            return personalized_content

        except Exception as e:
            self.logger.error(f"Error personalizing content for {user_id}: {e}")
            return content  # Return original content on error

    def _add_beginner_explanations(self, content: str) -> str:
        """Add beginner-friendly explanations to content"""
        # Add more detailed explanations for complex concepts
        beginner_additions = [
            ("[Complex concept]", "[Complex concept] - This means..."),
            ("algorithm", "algorithm - A step-by-step procedure for calculations"),
            ("framework", "framework - A platform for developing applications"),
            ("middleware", "middleware - Software that connects different applications")
        ]

        result = content
        for term, explanation in beginner_additions:
            result = result.replace(term, f"{explanation}")

        return result

    def _add_advanced_details(self, content: str) -> str:
        """Add advanced technical details to content"""
        # Add more technical depth
        return content  # In a real implementation, this would add more technical details

    def _enhance_visual_elements(self, content: str) -> str:
        """Enhance visual elements in content"""
        # Emphasize visual content
        return content  # In a real implementation, this would enhance visual elements

    def _add_hands_on_elements(self, content: str) -> str:
        """Add hands-on elements to content"""
        # Add more practical examples
        return content  # In a real implementation, this would add hands-on elements

    def _tailor_examples_by_interests(self, content: str, interests: list, context: Dict[str, Any]) -> str:
        """Tailor examples based on user interests"""
        # In a real implementation, this would customize examples based on interests
        return content

    async def get_personalized_recommendations(self, user_id: str, db: AsyncSession,
                                            context: Optional[Dict[str, Any]] = None) -> list:
        """Get personalized content recommendations for user"""
        try:
            user_prefs = await self.get_user_preferences(user_id, db)
            if not user_prefs:
                return []

            recommendations = []

            # Base recommendations on user's learning level and interests
            learning_level = user_prefs.get("learning_level", "beginner")
            interests = user_prefs.get("interests", [])
            completed_chapters = user_prefs.get("completed_chapters", [])

            # Generate recommendations based on preferences
            if learning_level == "beginner":
                recommendations.extend([
                    {"title": "Getting Started with ROS 2", "difficulty": "beginner", "relevance": 0.9},
                    {"title": "Understanding Robot Sensors", "difficulty": "beginner", "relevance": 0.8}
                ])
            elif learning_level == "intermediate":
                recommendations.extend([
                    {"title": "Advanced ROS 2 Concepts", "difficulty": "intermediate", "relevance": 0.9},
                    {"title": "Robot Control Systems", "difficulty": "intermediate", "relevance": 0.8}
                ])
            else:  # advanced
                recommendations.extend([
                    {"title": "Research-Level Robotics", "difficulty": "advanced", "relevance": 0.9},
                    {"title": "Advanced Control Theory", "difficulty": "advanced", "relevance": 0.8}
                ])

            # Filter out completed chapters
            recommendations = [
                rec for rec in recommendations
                if rec["title"] not in completed_chapters
            ]

            # Add topic-based recommendations
            for interest in interests:
                recommendations.append({
                    "title": f"Advanced {interest} Concepts",
                    "difficulty": learning_level,
                    "relevance": 0.7,
                    "interest_based": True
                })

            return recommendations

        except Exception as e:
            self.logger.error(f"Error getting recommendations for {user_id}: {e}")
            return []


# Example usage and testing
if __name__ == "__main__":
    import asyncio
    from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession

    async def test_personalization_service():
        # This is a basic test - in practice, you'd need a real database connection
        service = PersonalizationService()

        # Example usage would require a real database session
        print("Personalization service created successfully")

    asyncio.run(test_personalization_service())