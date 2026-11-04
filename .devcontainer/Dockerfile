FROM docker.io/ros:humble-ros-base

ENV IN_CONTAINER=1

# Add packages here
RUN apt-get update && apt-get install -y --no-install-recommends \ 
        clangd \
        bash-completion \
        ros-humble-pcl-ros \
        libasio-dev \
        clang && \
    rm -rf /var/lib/apt/lists/*
