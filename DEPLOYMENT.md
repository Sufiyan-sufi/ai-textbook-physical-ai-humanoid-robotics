# Deployment Guide for AI-Native Textbook on Physical AI & Humanoid Robotics

This guide provides detailed instructions for deploying the AI-Native Textbook application in various environments.

## Prerequisites

### Hardware Requirements
- **Minimum**: 8GB RAM, 4 CPU cores, 50GB disk space
- **Recommended**: 16GB+ RAM, 8+ CPU cores, 100GB+ disk space
- **GPU Support**: NVIDIA GPU with CUDA support (for Isaac Sim and advanced simulations)

### Software Requirements
- Ubuntu 22.04 LTS
- Docker Engine (v20.10+)
- Docker Compose (v2.0+)
- Git
- Python 3.11+
- Node.js 18+

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-textbook-physical-ai-humanoid-robotics
```

### 2. Configure Environment Variables
```bash
cp .env.example .env
```

Edit the `.env` file with your specific configuration:

```bash
# Database Configuration
DATABASE_URL=postgresql://postgres:postgres@db:5432/robotics_textbook

# Qdrant Configuration
QDRANT_URL=http://qdrant:6333

# API Keys
OPENAI_API_KEY=your_openai_api_key_here
SECRET_KEY=your_secret_key_here

# Application Configuration
API_BASE_URL=http://localhost:8000
FRONTEND_URL=http://localhost:3000
```

## Deployment Options

### Option 1: Local Development Deployment

For development and testing:

```bash
# Start all services
docker-compose up --build

# Access the application
# Frontend: http://localhost:3000
# Backend API: http://localhost:8000
# Qdrant UI: http://localhost:6333
```

### Option 2: Production Deployment with Docker (Recommended)

For production environments, use the production docker-compose configuration:

```bash
# Build and start services in detached mode using production config
docker-compose -f docker-compose.prod.yml up --build -d

# Check service status
docker-compose -f docker-compose.prod.yml ps

# Monitor logs
docker-compose -f docker-compose.prod.yml logs -f
```

### Option 3: Automated Production Deployment

Use the provided deployment script for simplified production deployment:

```bash
# Make the script executable
chmod +x deploy.sh

# Run the deployment script
./deploy.sh
```

The deployment script will:
- Validate all required environment variables
- Build and start production services
- Perform health checks
- Provide status updates

### Option 3: Cloud Deployment (AWS/GCP/Azure)

#### AWS ECS Deployment
1. Push Docker images to ECR
2. Create ECS cluster and services
3. Configure Application Load Balancer
4. Set up RDS for PostgreSQL
5. Configure security groups

#### GCP Cloud Run Deployment
1. Build container images
2. Push to Container Registry
3. Deploy to Cloud Run
4. Configure Cloud SQL for PostgreSQL

#### Azure Container Instances
1. Build and push container images
2. Deploy to Azure Container Instances
3. Configure Azure Database for PostgreSQL

## Service Configuration

### Backend Service
- **Port**: 8000
- **Workers**: 4 (auto-scaled based on load)
- **Timeout**: 120 seconds (for AI processing)
- **Max requests per worker**: 1000

### Frontend Service
- **Port**: 3000
- **Static assets**: Pre-built with gzip compression
- **Caching**: 1-hour cache for static assets

### Database Service (PostgreSQL)
- **Version**: 15
- **Persistence**: Volume-based storage
- **Backup**: Manual backup recommended

### Vector Database (Qdrant)
- **Port**: 6333 (HTTP), 6334 (gRPC)
- **Persistence**: Volume-based storage
- **Collection**: `textbook_content`

## Performance Tuning

### Backend Optimization
- Adjust worker count based on CPU cores: `workers = CPU_CORES * 2 + 1`
- Set appropriate timeout values for AI processing
- Configure connection pooling for database

### Frontend Optimization
- Enable gzip compression
- Set up proper caching headers
- Optimize image loading with lazy loading

### Database Optimization
- Use connection pooling
- Optimize queries with proper indexing
- Monitor and tune performance regularly

## Security Configuration

### HTTPS Setup
1. Obtain SSL certificate
2. Update nginx configuration with certificate paths
3. Redirect HTTP to HTTPS

### API Security
- Rate limiting: 100 requests/minute per IP
- Authentication for user-specific endpoints
- Input validation and sanitization

### Container Security
- Run containers as non-root user
- Use minimal base images
- Regular security updates

## Monitoring and Logging

### Application Logs
- Backend logs: `docker-compose logs backend`
- Frontend logs: `docker-compose logs frontend`
- Database logs: `docker-compose logs db`

### Health Checks
- Backend: `GET /health`
- Frontend: `GET /`
- Database: Connection test
- Qdrant: `GET /collections`

### Performance Metrics
- Enable nginx metrics endpoint
- Monitor container resource usage
- Set up alerting for critical metrics

## Backup and Recovery

### Database Backup
```bash
# Backup PostgreSQL
docker-compose exec db pg_dump -U postgres robotics_textbook > backup.sql

# Restore PostgreSQL
docker-compose exec -T db psql -U postgres robotics_textbook < backup.sql
```

### Vector Database Backup
- Qdrant data stored in named volume
- Backup volume data regularly
- Use Qdrant's snapshot feature for consistency

## Scaling

### Horizontal Scaling
- Increase backend workers in `gunicorn.conf.py`
- Add more frontend instances behind load balancer
- Scale database connections accordingly

### Vertical Scaling
- Increase container resources
- Upgrade hardware specifications
- Optimize application performance

## Troubleshooting

### Common Issues

#### 1. Container Won't Start
```bash
# Check container logs
docker-compose logs <service-name>

# Check system resources
docker stats
```

#### 2. API Timeouts
- Check OpenAI API key validity
- Verify network connectivity
- Increase timeout values if needed

#### 3. Database Connection Issues
- Verify database service is running
- Check connection string format
- Confirm database is properly initialized

### Performance Issues
- Monitor resource usage
- Check for bottlenecks in the system
- Optimize queries and API calls

## Updates and Maintenance

### Updating Application
```bash
# Pull latest code
git pull origin main

# Rebuild containers
docker-compose build --no-cache

# Restart services
docker-compose up -d
```

### Database Migrations
- Run migrations during deployment
- Test migrations in staging environment first
- Backup database before running migrations

## Hardware-Specific Configuration

### Jetson Platform
- Reduce worker count to conserve resources
- Optimize AI model usage for ARM architecture
- Adjust simulation complexity for embedded GPU

### RTX GPU Platforms
- Enable CUDA support for AI processing
- Configure Isaac Sim for GPU acceleration
- Optimize rendering settings for RTX capabilities

## Environment Variables Reference

| Variable | Description | Default |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://postgres:postgres@db:5432/robotics_textbook` |
| `QDRANT_URL` | Qdrant vector database URL | `http://qdrant:6333` |
| `OPENAI_API_KEY` | OpenAI API key for AI services | - |
| `SECRET_KEY` | Secret key for authentication | - |
| `API_BASE_URL` | Backend API base URL | `http://localhost:8000` |
| `FRONTEND_URL` | Frontend base URL | `http://localhost:3000` |
| `WORKERS` | Number of backend workers | 4 |
| `TIMEOUT` | Request timeout in seconds | 120 |

## Validation Checklist

Before going live, verify:

- [ ] All services are running
- [ ] Health checks pass
- [ ] Database connections work
- [ ] API endpoints are accessible
- [ ] Frontend loads correctly
- [ ] AI tutor responds to queries
- [ ] Simulations start successfully
- [ ] Personalization features work
- [ ] Translation works correctly
- [ ] Performance meets requirements
- [ ] Security configurations are applied
- [ ] Backup procedures are tested

## Support

For deployment issues or questions:

- Check the logs for error messages
- Verify all environment variables are set correctly
- Ensure hardware requirements are met
- Contact the development team if issues persist