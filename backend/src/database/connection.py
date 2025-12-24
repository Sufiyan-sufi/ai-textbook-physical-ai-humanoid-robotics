import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy.ext.asyncio import AsyncSession
import logging
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

# Database configuration
NEON_DB_URL = os.getenv("NEON_DATABASE_URL", os.getenv("DATABASE_URL", "postgresql+asyncpg://user:password@localhost:5432/textbook_db"))

# Ensure the URL uses the asyncpg driver if it's a PostgreSQL URL
if NEON_DB_URL.startswith("postgresql://"):
    DATABASE_URL = NEON_DB_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
else:
    DATABASE_URL = NEON_DB_URL

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=True,  # Set to False in production
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10,
    pool_recycle=300,
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Base class for SQLAlchemy models
class Base(DeclarativeBase):
    pass

async def init_db():
    """Initialize database connection and create tables"""
    # Import models to register them with SQLAlchemy
    from src.models.chapter import Chapter
    from src.models.user import User
    from src.models.tutorial import Tutorial
    from src.models.simulation import SimulationSession

    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(Base.metadata.create_all)
    logger.info("Database tables created successfully")

async def get_db():
    """Dependency to get database session"""
    async with AsyncSessionLocal() as session:
        yield session