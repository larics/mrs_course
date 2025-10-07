ARG ROS2_DISTRO=humble

# Use official ROS base image (multi-arch: amd64, arm64v8)
FROM ros:${ROS2_DISTRO} AS ros2_base

# Re-declare ARG variables after FROM for use in subsequent layers
ARG HOME=/root
ARG ROS2_WS=${HOME}/ros2_ws

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        nano \
        wget \
        htop \
        build-essential \
        tmux \
        tmuxinator \
        ranger \
        curl \
        openssh-client \
        libboost-python-dev \
        python3-pip \
        libfltk1.3-dev \
        libpng-dev \
        libjpeg-dev \
        libtool \
        libltdl-dev \
        libtool-bin \
        libgl1-mesa-dev \
        libglu1-mesa-dev

RUN pip install --upgrade pip

# Install Python dependencies
RUN pip install \
    pyyaml \
    numpy \
    scipy \
    pandas \
    matplotlib \
    plotly \
    networkx \
    tqdm \
    codetiming \
    rosbags

# Install ROS packages
RUN apt install -y \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt*

# Copy configuration files
COPY to_copy/tmux.conf ${HOME}/.tmux.conf
COPY to_copy/aliases ${HOME}/.bash_aliases
COPY to_copy/nanorc ${HOME}/.nanorc
COPY to_copy/ranger ${HOME}/.config/ranger/rc.conf

# Enable SSH for git cloning
RUN --mount=type=ssh id=default mkdir -p ~/.ssh/ && ssh-keyscan -H github.com >> ~/.ssh/known_hosts

# Set up workspaces
RUN mkdir -p ${ROS2_WS}/src

# Clone ROS2 repositories
WORKDIR ${ROS2_WS}/src
RUN git clone https://github.com/tuw-robotics/Stage.git
RUN git clone https://github.com/tuw-robotics/stage_ros2.git
RUN git clone https://github.com/larics/sphero_simulation.git

# Build ROS2 workspace
RUN rosdep update
RUN rosdep install --from-paths ./Stage --ignore-src -r -y
RUN rosdep install --from-paths ./stage_ros2 --ignore-src -r -y
WORKDIR ${ROS2_WS}
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# Add ROS2 workspace setup to .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc
RUN echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> $HOME/.bashrc

# Clean up apt cache for smaller image
RUN rm -rf /var/lib/apt/lists/*

# Final settings
WORKDIR ${ROS2_WS}/src
ENTRYPOINT []
CMD ["/bin/bash"]
