# Gunicorn configuration file for AI-Native Textbook backend

# Server socket
bind = "0.0.0.0:8000"
backlog = 2048

# Worker processes
workers = 4
worker_class = "uvicorn.workers.UvicornWorker"
worker_connections = 1000
max_requests = 1000
max_requests_jitter = 100
timeout = 120
keepalive = 5

# Restart workers after this many requests, to help prevent memory leaks
max_worker_connections = 1000

# Logging
accesslog = "-"
errorlog = "-"
loglevel = "info"
access_log_format = '%(h)s %(l)s %(u)s %(t)s "%(r)s" %(s)s %(b)s "%(f)s" "%(a)s" %(D)s'

# Process naming
proc_name = 'robotics_textbook_backend'

# Server mechanics
preload_app = True
daemon = False
pidfile = '/tmp/robotics_textbook_backend.pid'
user = None
group = None
tmp_upload_dir = None

# Security and resource limits
limit_request_line = 4094
limit_request_fields = 100
limit_request_field_size = 8190

# Performance tuning for AI services
worker_tmp_dir = "/dev/shm"  # Use memory for temporary files if available