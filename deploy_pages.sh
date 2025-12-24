#!/bin/bash

# GitHub Pages Deployment Script

echo "Starting GitHub Pages deployment..."

# Navigate to the website directory
cd /mnt/d/hackathon-book-2025/my-website

# Build the site if not already built
if [ ! -d "build" ]; then
    echo "Building the site..."
    npm run build
fi

# Check if build was successful
if [ ! -d "build" ]; then
    echo "Build failed. Exiting."
    exit 1
fi

echo "Build completed successfully."

# Create a temporary directory for deployment
TEMP_DIR=$(mktemp -d)

# Copy the built files to the temporary directory
cp -r build/* $TEMP_DIR/

# Navigate to the temporary directory
cd $TEMP_DIR

# Initialize git
git init
git remote add origin https://github.com/Sufiyan-sufi/ai-textbook-physical-ai-humanoid-robotics.git

# Configure git
git config user.name "Claude"
git config user.email "claude@anthropic.com"

# Add all files
git add .

# Create a commit
git commit -m "Deploy to GitHub Pages"

# Push to gh-pages branch
git push -f origin main:gh-pages

# Clean up
rm -rf $TEMP_DIR

echo "GitHub Pages deployment completed!"