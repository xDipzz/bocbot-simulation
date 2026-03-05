FROM osrf/ros:eloquent-desktop

# Install additional required packages
RUN apt-get update && apt-get install -y ros-eloquent-gazebo-ros-pkgs ros-eloquent-teleop-twist-keyboard ros-eloquent-xacro xterm sudo && rm -rf /var/lib/apt/lists/*

# Add user matching host user to avoid permission issues with mounted volumes
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} dipz && useradd -m -u ${UID} -g ${GID} -s /bin/bash -G sudo dipz && echo "dipz ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dipz
WORKDIR /home/dipz/robot_proj/bocbot_ws

# Source ROS 2 environment in bashrc
RUN echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /home/dipz/robot_proj/bocbot_ws/install/setup.bash ]; then source /home/dipz/robot_proj/bocbot_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]