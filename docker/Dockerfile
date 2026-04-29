FROM erdemuysalx/px4-sitl:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        libompl-dev \
        ros-jazzy-octomap \
        ros-jazzy-octomap-msgs \
        ros-jazzy-octomap-ros \
        ros-jazzy-octomap-rviz-plugins \
        ros-jazzy-octomap-server \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages --ignore-installed typing_extensions mavsdk

WORKDIR /root/3d-path-planner
