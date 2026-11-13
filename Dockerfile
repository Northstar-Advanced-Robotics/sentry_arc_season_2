ARG ROS_DISTRO=jazzy
FROM docker.io/ros:${ROS_DISTRO}-ros-base as base

FROM base as overlay

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
&&  cd ./Livox-SDK2/ \
&&  git checkout c0796f04c143143899c87a773d9f6b7136453c0b \
&&  mkdir build \
&&  cd build \
&&  cmake .. && make -j \
&&  make install

RUN apt-get update && apt-get install -y --no-install-recommends \
        symlinks \
&&  mkdir -p /overlay/src \
&&  cd overlay/src \
&&  git clone --branch 1.2.6 https://github.com/Livox-SDK/livox_ros_driver2.git \ 
&&  . /opt/ros/${ROS_DISTRO}/setup.sh \ 
&&  cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml \
&&  rosdep install --from-paths . --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
&&  cd livox_ros_driver2 && ./build.sh ${ROS_DISTRO} \
&&  symlinks -cr / > /dev/null 2>&1 \
&&  rm -rf /var/lib/apt/lists/*

FROM base as dev 

ENV DEBIAN_FRONTEND=noninteractive \
    WEBOTS_HOME=/usr/local/webots \
    USER=root

RUN mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && curl -OL https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/Cyberbotics.list && \
    apt-get update && apt-get install -y \
      webots && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        clangd \
        ninja-build \
        bash-completion \
        clang && \
    rm -rf /var/lib/apt/lists/*

COPY --from=overlay /overlay/install /livox_ros_driver2/install
COPY --from=overlay /overlay/src/livox_ros_driver2/package.xml /livox_ros_driver2/package.xml
COPY --from=overlay /usr/local /usr/local

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
&&  echo 'source /livox_ros_driver2/install/setup.bash' >> /root/.bashrc \
&&  echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /root/.bashrc

RUN apt-get update && \
    rosdep install --from-paths /livox_ros_driver2 -y --ignore-src --dependency-types exec && \
    rm -rf /var/lib/apt/lists/*

COPY --parents src/**/package.xml /ws/

RUN apt-get update && \ 
    rosdep install --from-paths /ws --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

ENV IN_CONTAINER=1
