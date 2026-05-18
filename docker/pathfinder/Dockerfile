FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-suite \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-tf2-ros \
    ros-noetic-turtlebot3-description \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
