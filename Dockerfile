# =============================================================================
# Dockerfile: ROS2 Humble + Gazebo + TurtleBot3 for Path Smoothing Assignment
# =============================================================================
# Base: Official ROS2 Humble desktop (includes RViz2, Gazebo, dev tools)
FROM osrf/ros:humble-desktop

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# ---- Install TurtleBot3 + Extra Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-plotjuggler-ros \
    ros-humble-rqt* \
    libeigen3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    clang-format \
    cppcheck \
    vim \
    nano \
    htop \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ---- Python plotting/analysis tools ----
RUN pip3 install --no-cache-dir \
    matplotlib \
    numpy \
    scipy \
    transforms3d \
    pandas

# ---- Set TurtleBot3 model ----
ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}

# ---- ROS2 workspace setup ----
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# ---- Source ROS2 in every shell ----
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# ---- Entry point ----
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
