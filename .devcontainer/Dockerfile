FROM docker.io/ros:humble-ros-base as base

FROM base as overlay

RUN apt-get update && apt-get install -y --no-install-recommends \
        symlinks \
        ros-humble-pcl-ros && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /overlay/src \
&&  cd overlay/src \
&&  git clone https://github.com/Livox-SDK/livox_ros2_driver.git \ 
&&  cd .. && . /opt/ros/humble/setup.sh && colcon build --symlink-install \
&&  symlinks -cr /

FROM base as dev 

COPY --from=overlay overlay /livox_ros2_driver

RUN apt-get update && apt-get install -y --no-install-recommends \ 
        clangd \
        bash-completion \
        libasio-dev \
        clang && \
    rm -rf /var/lib/apt/lists/*

RUN echo 'source /livox_ros2_driver/install/setup.bash' >> /root/.bashrc


ENV IN_CONTAINER=1
