FROM moyash/novnc-ros:custom

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3


SHELL ["/bin/bash", "-c"]

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        iputils-ping \
        traceroute \
        vim \
        nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        python3-rosdep \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-tf \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
COPY --chown=1000:1000 src .
COPY realsense-ros/realsense2_description realsense2_description
WORKDIR /catkin_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    catkin build

# RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc # already done in src image
RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc