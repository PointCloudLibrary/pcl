ARG flavor="melodic"

ARG workspace="/root/catkin_ws"

FROM ros:${flavor}-ros-core
COPY ${flavor}_rosinstall.yaml ${workspace}/src/.rosinstall

# Be careful, ROS uses Python2
# The dependencies of PCL can be reduced since
# * we don't need to build visualization or docs
RUN sed -i "s/^# deb-src/deb-src/" /etc/apt/sources.list \
 && apt update \
 && apt install -y \
    libeigen3-dev \
    libflann-dev \
    python-pip \
 && apt build-dep pcl -y \
 && pip install -U pip \
 && pip install catkin_tools \
 && cd ${workspace}/src \
 && . "/opt/ros/${flavor}/setup.sh" \
 && catkin_init_workspace \
 && cd .. \
 && wstool update -t src

COPY package.xml ${workspace}/src/pcl/

RUN cd ${workspace} \
 && catkin build libpcl-all-dev --cmake-args -DWITH_OPENGL:BOOL=OFF \
 && catkin build
