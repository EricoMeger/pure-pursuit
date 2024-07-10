# Pure Pursuit

Pure Pursuit path tracking algorithm implemented in Python using ROS as middleware.

## Overview

This repository contains an implementation of the Pure Pursuit algorithm for path tracking. The algorithm requires the `follow_path` and `vel_limiter` nodes to be running for proper operation.

### `vel_limiter`

The `vel_limiter` node limits the robot's velocity. It caps the robot's velocity at 0.4 m/s and 0.4 rad/s, and uses LiDAR readings to stop the robot if there is an obstacle nearby. This behavior is based on the `/range_min` topic, which publishes the smallest range within the 90 degrees in front of the LiDAR.

`vel_limiter` is optional. If you do not want to cap your robot's velocity, you can change the topic that `follow_path` publishes its velocities to `cmd_vel`.

## Installation

To run the algorithm, follow these steps:

1. Clone this git repository into your workspace `src` directory:
   ```bash
   git clone https://github.com/EricoMeger/pure-pursuit
   ```
2. Build the workspace:
    ```bash
    catkin_make
    ```
3. Launch the Pure Pursuit algorithm:
    ```bash
    roslaunch pure-pursuit pursuit.launch
    ```

## Usage

`follow_path` will start publishing velocities immediately after receiving a path message.
