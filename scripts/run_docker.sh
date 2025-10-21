#!/bin/bash

# Allow X server connection
xhost +local:docker

# Build and run
docker-compose up --build

# Cleanup
xhost -local:docker
