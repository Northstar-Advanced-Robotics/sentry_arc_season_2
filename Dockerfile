ARG ROS_DISTRO=jazzy
FROM docker.io/ros:${ROS_DISTRO}-ros-base as base

FROM base as overlay

RUN apt-get update && apt-get install -y --no-install-recommends \
        symlinks \
        ros-${ROS_DISTRO}-pcl-ros && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
&& cd ./Livox-SDK2/ \
&& mkdir build \
&& cd build \
&& cmake .. && make -j \
&& make install

RUN mkdir -p /overlay/src \
&&  cd overlay/src \
&&  git clone https://github.com/Livox-SDK/livox_ros_driver2.git \ 
&&  . /opt/ros/${ROS_DISTRO}/setup.sh \ 
&&  cd livox_ros_driver2 && ./build.sh jazzy \
&&  symlinks -cr / > /dev/null 2>&1

FROM base as dev 

COPY --from=overlay overlay /livox_ros_driver2
COPY --from=overlay /usr/local /usr/local
RUN echo 'source /livox_ros_driver2/install/setup.bash' >> /root/.bashrc \
&&  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root.bashrc 


RUN apt-get update && apt-get install -y --no-install-recommends \ 
        clangd \
        bash-completion \
        libasio-dev \
        clang && \
    rm -rf /var/lib/apt/lists/*

ENV IN_CONTAINER=1
