ARG ROS_DISTRO=jazzy
FROM docker.io/ros:${ROS_DISTRO}-ros-base as base

FROM base as overlay

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
&& cd ./Livox-SDK2/ \
&& mkdir build \
&& cd build \
&& cmake .. && make -j \
&& make install

RUN apt-get update && apt-get install -y --no-install-recommends \
        symlinks \
&&  mkdir -p /overlay/src \
&&  cd overlay/src \
&&  git clone https://github.com/Livox-SDK/livox_ros_driver2.git \ 
&&  . /opt/ros/${ROS_DISTRO}/setup.sh \ 
&&  cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml \
&&  rosdep install --from-paths . --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
&&  cd livox_ros_driver2 && ./build.sh ${ROS_DISTRO} \
&&  symlinks -cr / > /dev/null 2>&1 \
&&  rm -rf /var/lib/apt/lists/*

FROM base as dev 

COPY --from=overlay /overlay/install /livox_ros_driver2/install
COPY --from=overlay /usr/local /usr/local
COPY --parents src/**/package.xml /ws/

RUN echo 'source /livox_ros_driver2/install/setup.bash' >> /root/.bashrc \
&&  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
&&  echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /root/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \ 
        clangd \
        ninja-build \
        bash-completion \
        clang && \
    rosdep install --from-paths /ws --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

ENV IN_CONTAINER=1
