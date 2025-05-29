# ROS Wall Following and Odometry Recording Package

This repository contains a ROS (Robot Operating System) Noetic package designed to enable a mobile robot to perform wall-following behavior and record its odometry data. This project focuses on implementing fundamental robot control using ROS topics, services, and actions, with an emphasis on simulation-first development.


## Project Overview

The primary goal of this ROS package is to teach a robot to navigate an environment by following walls on its right-hand side, maintaining a consistent distance. Additionally, it implements a service to help the robot find the nearest wall before starting the following behavior, and an action to record the robot's odometry data throughout its mission. The development process emphasizes testing in simulation before deployment on a real robot.

## Functionality

### Wall Following Behavior

The core behavior is implemented in `wall_follower_node.py`[cite: 5]. The robot maintains a distance of approximately 30cm from a wall on its right-hand side[cite: 2].

* **Input:** Subscribes to the `/scan` laser topic (of type `sensor_msgs/LaserScan`) to measure the robot's distance to the wall using the rightmost laser ray (90º angle to the right with the front of the robot)[cite: 3]. It also utilizes the front laser ray to detect upcoming obstacles[cite: 5].
* **Output:** Publishes to the `/cmd_vel` velocity topic (of type `geometry_msgs/Twist`) to control the robot's linear and angular speed[cite: 4, 5].
* **Logic:**
    * If the right ray distance is `> 0.3m`, the robot adds some rotational speed to approach the wall[cite: 4].
    * If the right ray distance is `< 0.2m`, the robot adds rotational speed in the opposite direction to move away from the wall[cite: 4].
    * If the right ray distance is `between 0.2m and 0.3m`, the robot moves forward without additional rotational speed[cite: 4].
    * **Wall Crossing Behavior:** If the front laser ray detects an obstacle closer than `0.5m`, the robot turns fast to the left while moving forward, transitioning to follow the next wall[cite: 4].

### Find Wall Service

A service server (`find_wall_server.py`) [cite: 6] is created to enable the robot to autonomously search for and align with the nearest wall before commencing the wall-following behavior.

* **Service Message:** Uses a custom service message `FindWall.srv`[cite: 7], which has a boolean response `wallfound`.
    ```
    ---
    bool wallfound
    ```
* **Behavior when called:**
    1.  Identifies the shortest laser ray (assumed to be pointing at a wall)[cite: 7].
    2.  Rotates the robot until its front faces the identified wall[cite: 7].
    3.  Moves the robot forward until the front ray is shorter than 30cm[cite: 7].
    4.  Rotates the robot again until ray number 270 (assuming it corresponds to the right-hand side) points to the wall[cite: 7].
    5.  Returns `wallfound: True`[cite: 7].
* The `wall_follower_node.py` includes a service client that calls this `find_wall` service once at startup[cite: 5].

### Record Odometry Action

An action server (`record_odom_server.py`) is implemented to continuously record the robot's odometry (position and orientation) during its movement.

* **Action Message:** Uses a custom action message `OdomRecord.action`[cite: 8].
    ```
    # Goal
    ---
    # Result
    geometry_msgs/Point32[] list_of_odoms
    ---
    # Feedback
    float32 current_total
    ```
* **Behavior:**
    * Starts recording (x, y, theta) odometry as a `Point32` every second.
    * Provides feedback on the `current_total` distance the robot has moved so far.
    * Finishes and returns the `list_of_odoms` when the robot completes a full lap.
* The `wall_follower_node.py` also includes an action client that calls this `record_odom` action server at startup.

## ROS Components

* `action/OdomRecord.action`: Defines the custom action message for odometry recording.
* `launch/main.launch`: A ROS launch file that starts both the `find_wall_server.py` service node, the `record_odom_server.py` action node, and the `wall_follower_node.py` control node[cite: 10].
* `scripts/find_wall_server.py`: The Python ROS node implementing the `find_wall` service server.
* `scripts/record_odom_server.py`: The Python ROS node implementing the `record_odom` action server.
* `scripts/wall_follower_node.py`: The main Python ROS node implementing the wall-following logic and containing clients for the `find_wall` service and `record_odom` action[cite: 5].
* `srv/FindWall.srv`: Defines the custom service message for the `find_wall` service.
* `CMakeLists.txt`: CMake build script for the ROS package.
* `package.xml`: ROS package manifest file, defining package metadata and dependencies.

## System Requirements

* **Operating System:** Ubuntu (typically recommended for ROS)
* **ROS Distribution:** ROS Noetic [cite: 1]
* **Python:** Python 3 (as indicated by `#!/usr/bin/env python3` in `wall_follower_node.py`)
* **ROS Packages:**
    * `rospy`
    * `sensor_msgs` (for `LaserScan`)
    * `geometry_msgs` (for `Twist`, `Point32`)
    * `actionlib` and `actionlib_msgs` (for ROS actions)
    * `std_msgs` (for Header in LaserScan)

## Installation

1.  **Create a ROS Workspace (if you don't have one):**
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```
2.  **Clone this repository into your workspace's `src` folder:**
    ```bash
    git clone https://github.com/rawanbasem/Wall-Follower-TurtleBot3.git
    ```
    *Note: If your local project folder is already named `wall_follower` (or similar), ensure it matches your package name in `package.xml`.*
3.  **Build the package:**
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
4.  **Source your workspace:**
    ```bash
    source devel/setup.bash
    ```
    *It's recommended to add this line to your `~/.bashrc` file for automatic sourcing.*

## Usage

### Starting the Simulation

Ensure your robot simulation environment is launched as per your course instructions. For example, if it's a Gazebo simulation for a Kobuki-like robot, you would typically run a command similar to:
```bash
roslaunch <your_robot_gazebo_package> <your_robot_world>.launch
