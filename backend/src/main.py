from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
import logging
import os

from src.api.v1 import textbook_routes, ai_tutor_routes, simulation_routes, ingest

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="AI-Native Textbook API",
    description="Backend API for AI-Native Textbook on Physical AI & Humanoid Robotics",
    version="1.0.0"
)

# Add TrustedHost middleware for production
if os.getenv("ENVIRONMENT") == "production":
    # Production: Only allow specific hosts
    allowed_hosts = os.getenv("ALLOWED_HOSTS", "").split(",") if os.getenv("ALLOWED_HOSTS") else []
    if not allowed_hosts:
        # Use the frontend URL host as default
        frontend_url = os.getenv("FRONTEND_URL", "https://yourdomain.com")
        allowed_hosts = [frontend_url.replace("https://", "").replace("http://", "")]
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=allowed_hosts)

# Configure CORS based on environment
if os.getenv("ENVIRONMENT") == "production":
    # Production: Restrict to specific origins
    allowed_origins = os.getenv("ALLOWED_ORIGINS", "").split(",") if os.getenv("ALLOWED_ORIGINS") else []
    if not allowed_origins:
        allowed_origins = os.getenv("FRONTEND_URL", "https://yourdomain.com").split(",")  # Use FRONTEND_URL as default, fallback to example
else:
    # Development: Allow all origins
    allowed_origins = ["*"]

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(textbook_routes.router, prefix="/api/v1/textbook", tags=["textbook"])
app.include_router(ai_tutor_routes.router, prefix="/api/v1/ai-tutor", tags=["ai-tutor"])
app.include_router(simulation_routes.router, prefix="/api/v1/simulation", tags=["simulation"])
app.include_router(ingest.router, prefix="/api/v1/ingest", tags=["ingest"])

@app.get("/")
def read_root():
    return {"message": "AI-Native Textbook Backend API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "textbook-backend"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)