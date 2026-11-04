FROM docker.io/ros:jazzy-ros-base

ENV IN_CONTAINER=1

# Add packages here
RUN apt-get update && apt-get install -y --no-install-recommends \ 
        clangd \
        bash-completion \
        clang && \
    rm -rf /var/lib/apt/lists/*
