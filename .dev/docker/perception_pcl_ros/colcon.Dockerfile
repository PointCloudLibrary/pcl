# flavor appears twice, once for the FOR, once for the contents since
# Dockerfile seems to reset arguments after a FOR appears
ARG flavor="dashing"
FROM ros:${flavor}-ros-base

ARG flavor="dashing"
ARG workspace="/root/catkin_ws"

COPY ${flavor}_rosinstall.yaml ${workspace}/src/.rosinstall

# Be careful:
# * source ROS setup file in evey RUN snippet
#
# TODO: The dependencies of PCL can be reduced since
# * we don't need to build visualization or docs
RUN sed -i "s/^# deb-src/deb-src/" /etc/apt/sources.list \
 && apt update \
 && apt install -y \
    libeigen3-dev \
    libflann-dev \
    ros-${flavor}-tf2-eigen \
 && apt build-dep pcl -y \
 && pip3 install -U pip \
 && pip3 install wstool \
 && cd ${workspace}/src \
 && . "/opt/ros/${flavor}/setup.sh" \
 && catkin_init_workspace \
 && cd .. \
 && wstool update -t src

COPY package.xml ${workspace}/src/pcl/

RUN cd ${workspace} \
 && . "/opt/ros/${flavor}/setup.sh" \
 && catkin config --install \
 && catkin build -j2 libpcl-all-dev --cmake-args -DWITH_OPENGL:BOOL=OFF \
 && catkin build
