from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from src.api.v1 import textbook_routes, ai_tutor_routes, simulation_routes

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="AI-Native Textbook API",
    description="Backend API for AI-Native Textbook on Physical AI & Humanoid Robotics",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(textbook_routes.router, prefix="/api/v1/textbook", tags=["textbook"])
app.include_router(ai_tutor_routes.router, prefix="/api/v1/ai-tutor", tags=["ai-tutor"])
app.include_router(simulation_routes.router, prefix="/api/v1/simulation", tags=["simulation"])

@app.get("/")
def read_root():
    return {"message": "AI-Native Textbook Backend API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "textbook-backend"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)