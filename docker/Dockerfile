FROM osrf/ros:noetic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive

# Install things that need root first
RUN apt-get update && apt-get install -y \
        sudo vim git fonts-font-awesome curl wget zsh tmux \
        python3-pip python3-rosdep python3-rosinstall \
        python3-rosinstall-generator python3-wstool build-essential \
	mesa-utils libgl1-mesa-dri libglx-mesa0 \
	ros-noetic-turtlebot3 ros-noetic-turtlebot3-description ros-noetic-turtlebot3-simulations \	
 && rm -rf /var/lib/apt/lists/*

# Create an "ubuntu" user with UID/GID 1000 to match your host user
# (osrf/ros:noetic images don't ship one by default, unlike the Jazzy image)
RUN groupadd -g 1000 ubuntu \
 && useradd -m -u 1000 -g 1000 -s /bin/zsh ubuntu \
 && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
 && chmod 0440 /etc/sudoers.d/ubuntu \
 && mkdir -p /app && chown ubuntu:ubuntu /app

USER ubuntu
WORKDIR /home/ubuntu/workspace

# Now run the user-specific setup as ubuntu
COPY --chown=ubuntu:ubuntu install-zsh.sh /tmp/install-zsh.sh
RUN chmod +x /tmp/install-zsh.sh && /tmp/install-zsh.sh && rm /tmp/install-zsh.sh

# Source ROS 1 in the shell
RUN echo 'source /opt/ros/noetic/setup.zsh' >> /home/ubuntu/.zshrc

SHELL ["/bin/zsh", "-c"]
CMD ["/bin/zsh"]
