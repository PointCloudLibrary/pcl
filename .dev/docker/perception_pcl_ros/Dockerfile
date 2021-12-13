# flavor appears twice, once for the FOR, once for the contents since
# Dockerfile seems to reset arguments after a FOR appears
ARG flavor="melodic"
FROM ros:${flavor}-robot

ARG flavor="melodic"
ARG workspace="/root/catkin_ws"

COPY ${flavor}_rosinstall.yaml ${workspace}/src/.rosinstall

# Be careful:
# * ROS uses Python2
# * source ROS setup file in evey RUN snippet
#
# The dependencies of PCL can be reduced since
# * we don't need to build visualization or docs
RUN sed -i "s/^# deb-src/deb-src/" /etc/apt/sources.list \
 && apt update \
 && apt install -y \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    python-pip \
    ros-${flavor}-tf2-eigen \
 && pip install -U pip \
 && pip install catkin_tools \
 && cd ${workspace}/src \
 && . "/opt/ros/${flavor}/setup.sh" \
 && catkin_init_workspace \
 && cd .. \
 && wstool update -t src

COPY package.xml ${workspace}/src/pcl/

RUN cd ${workspace} \
 && . "/opt/ros/${flavor}/setup.sh" \
 && catkin config --install --link-devel \
 && catkin build --no-status --verbose --summary -j2 libpcl-all-dev --cmake-args -DWITH_OPENGL:BOOL=OFF \
 && rm -fr build/libpcl-all-dev \
 && catkin build --no-status --verbose --summary --start-with pcl_msgs
