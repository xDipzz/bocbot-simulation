#!/bin/bash

set -euo pipefail

# Allow local X11 connections for GUI apps (Gazebo)
if command -v xhost &>/dev/null && [ -n "${DISPLAY:-}" ]; then
    xhost +local:
else
    echo "[WARN] No X11 display or xhost not found — skipping xhost setup"
fi

docker_args=(
    -it
    --rm
    --name bocbot_container
    --net host
    --env DISPLAY
    --env QT_X11_NO_MITSHM=1
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
    --volume "$PWD/bocbot_ws:/home/dipz/robot_proj/bocbot_ws:rw"
)

# Check for GPU device
if [ -e /dev/dri ]; then
    docker_args+=(--device /dev/dri:/dev/dri)
fi

# Build the docker image
echo "Building the Docker image..."
docker build -t bocbot_env -f Dockerfile .

# Run the docker container with X11 forwarding and current directory mounted
echo "Starting the Docker container..."
if [ $# -eq 0 ]; then
    docker run "${docker_args[@]}" bocbot_env bash -lc \
        "source /opt/ros/eloquent/setup.bash && \
        colcon build && \
        source install/setup.bash && \
        echo 'Workspace built successfully!' && \
        echo 'To launch the simulation, run: ros2 launch bocbot world.launch.py' && \
        exec bash"
else
    docker run "${docker_args[@]}" bocbot_env bash -lc \
        'source /opt/ros/eloquent/setup.bash && \
        colcon build && \
        source install/setup.bash && \
        exec "$@"' \
        bash "$@"
fi
