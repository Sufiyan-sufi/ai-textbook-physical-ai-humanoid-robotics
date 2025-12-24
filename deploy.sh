#!/bin/bash
# Deployment script for AI-Native Textbook Backend

set -e  # Exit on any error

echo "AI-Native Textbook Backend Deployment Script"
echo "============================================"

# Check if we're in the right directory
if [ ! -f "docker-compose.prod.yml" ]; then
    echo "Error: docker-compose.prod.yml not found in current directory"
    echo "Please run this script from the project root directory"
    exit 1
fi

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo "Error: .env file not found"
    echo "Please create a .env file with all required environment variables"
    exit 1
fi

# Load environment variables
source .env

echo "Validating environment variables..."
required_vars=(
    "DATABASE_URL"
    "QDRANT_URL"
    "QDRANT_API_KEY"
    "QDRANT_CLUSTER_URL"
    "OPENAI_API_KEY"
    "COHERE_API_KEY"
    "NEON_DATABASE_URL"
    "SECRET_KEY"
    "POSTGRES_PASSWORD"
)

missing_vars=()
for var in "${required_vars[@]}"; do
    if [ -z "${!var}" ]; then
        missing_vars+=("$var")
    fi
done

if [ ${#missing_vars[@]} -ne 0 ]; then
    echo "Error: The following required environment variables are not set:"
    printf '%s\n' "${missing_vars[@]}"
    exit 1
fi

echo "All required environment variables are set."

# Build and start services
echo "Building and starting services..."
docker-compose -f docker-compose.prod.yml up --build -d

echo "Waiting for services to start..."
sleep 30

# Check health of backend
echo "Checking backend health..."
if curl -f http://localhost:8000/health > /dev/null 2>&1; then
    echo "✓ Backend is healthy"
else
    echo "⚠ Backend health check failed, checking logs..."
    docker-compose -f docker-compose.prod.yml logs backend
fi

echo ""
echo "Deployment completed!"
echo "Services are running:"
echo "- Backend API: http://localhost:8000"
echo "- Health check: http://localhost:8000/health"
echo ""
echo "To view logs: docker-compose -f docker-compose.prod.yml logs -f"
echo "To stop services: docker-compose -f docker-compose.prod.yml down"