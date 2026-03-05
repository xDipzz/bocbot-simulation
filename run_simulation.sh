#!/bin/bash

# Allow local X11 connections for GUI apps (Gazebo)
xhost +local:root

# Build the docker image
echo "Building the Docker image..."
docker build -t bocbot_env -f Dockerfile .

# Run the docker container with X11 forwarding and current directory mounted
echo "Starting the Docker container..."
if [ $# -eq 0 ]; then
    # No arguments, drop to bash
    docker run -it --rm --name bocbot_container --net host --device=/dev/dri:/dev/dri --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$PWD/bocbot_ws:/home/dipz/robot_proj/bocbot_ws:rw" bocbot_env bash -c "source /opt/ros/eloquent/setup.bash && colcon build && source install/setup.bash && echo 'Workspace built successfully!' && echo 'To launch the simulation, run: ros2 launch bocbot world.launch.py' && bash"
else
    # Execute the command passed as arguments
    docker run -it --rm --name bocbot_container --net host --device=/dev/dri:/dev/dri --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$PWD/bocbot_ws:/home/dipz/robot_proj/bocbot_ws:rw" bocbot_env bash -c "source /opt/ros/eloquent/setup.bash && colcon build && source install/setup.bash && $*"
fi