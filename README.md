# Custom ROS 2 Gazebo Robot Simulation 🤖

This repository contains a full physics simulation of a custom ROS 2 robot operating inside a simulated Gazebo environment. 

Because modern host machines no longer natively support ROS 2 Eloquent or Gazebo Classic 11, this project uses a deeply integrated **Docker + X11 Forwarding** setup. This guarantees the simulation will run flawlessly on any Linux host (like Ubuntu 24.04) without causing package conflicts.

## 🚀 Features
- **Containerized Environment:** Fully isolated `osrf/ros:eloquent-desktop` environment.
- **Custom Physics Model:** A 4-wheel skid-steer robot model generated via Xacro/URDF with integrated camera and LiDAR (Hokuyo) sensors.
- **Office Environment:** A custom 3D office world mapped out in Gazebo.
- **Gaming-style RC Controller:** A custom-built, ultra-smooth WASD python teleop script featuring simultaneous key inputs, acceleration/deceleration ramps, and instant teleport resets.

---

## 🛠️ How to Run the Simulation

You only need Docker installed on your host machine to run this project! 

### 1. Build and Launch the Container
Open a terminal in the root of this project and run the provided startup script. This will set up the X11 graphics passthrough, build the Docker image, compile the ROS 2 workspace, and drop you into the container's interactive shell.

```bash
./run_simulation.sh
```

### 2. Start the Simulation
Once the script finishes and you are inside the `bocbot_env` container shell, launch the world:

```bash
ros2 launch bocbot world.launch.py
```

Gazebo will automatically open displaying the office environment and the robot.

### 3. Drive the Robot
When the simulation launches, a small black terminal window titled **"Robot Controller (WASD)"** will automatically pop up. 

Click inside that window and use your keyboard to drive:
* **W** - Accelerate Forward
* **S** - Reverse
* **A / D** - Steer Left / Steer Right (Can be combined with W or S!)
* **R** - Rescue/Reset (Instantly teleports the robot back to the starting point if you get stuck or flip over).
* **Q / E** - Pivot in place.

*(Note: Release all keys to automatically apply the brakes).*

---

## 📁 Repository Structure
* `/Dockerfile` - The blueprint for the ROS 2 Eloquent + Gazebo 11 environment.
* `/run_simulation.sh` - The automated startup script handling Docker builds and X11 GUI forwarding.
* `/bocbot_ws/` - The ROS 2 Workspace containing the packages.
  * `/src/bocbot/urdf/` - Contains the Xacro, XML, and Gazebo plugin definitions for the physical robot.
  * `/src/bocbot/worlds/` - Contains the SDF layout of the simulated office building.
  * `/src/bocbot/launch/` - Contains the Python launch script orchestrating the simulation spawn.
  * `/src/bocbot/src/teleop_wasd.py` - The custom RC-style driving script.
