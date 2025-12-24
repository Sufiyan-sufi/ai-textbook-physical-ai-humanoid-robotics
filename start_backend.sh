#!/bin/bash
cd /mnt/d/hackathon-book-2025/backend
source venv/bin/activate

# Load environment variables from .env file
while IFS= read -r line; do
    if [[ $line != \#* ]] && [[ -n $line ]]; then
        export "$line"
    fi
done < ../.env

# Start the server
nohup python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload > server.log 2>&1 &
echo "Backend server started with PID $!"