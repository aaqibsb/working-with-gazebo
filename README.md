# ROS2 Humble - Simple Walker Algorithm for Turtlebot3

## Overview

This ROS2 workspace contains one package:

- walker: Moves the Turtlebot3 robot based on laser scan data

Results folder has the cpplint and cppcheck outputs along with the rosbag recording.

## Dependencies

Please make sure the following prerequisites are met before running this package on your system:

- ROS2 Humble
- Ubuntu 22.04
- VSCode


## Build and Run Instructions

1. **Clone Package**

   Clone the repository/package by using the following command in the terminal. Make sure to place it in the `src` directory of your workspace.

   ```sh
   cd <your_ros2_workspace>/src
   git clone https://github.com/aaqibsb/working-with-gazebo.git
   ```

2. **Build the Package**

    Change the directory to your ROS2 workspace and build the package using colcon.

    ```sh
    cd ..
    colcon build
    ```

3. **Source the Workspace**

    Source the ROS2 workspace to set the environment for the package.

    ```sh
    source install/setup.bash
    ```

5. **Run the launch file**

    Run the following command to start the simulation in Gazebo.

    ```sh
    ros2 launch walker walk_launch.py
    ```

6. **Run the launch file with rosbag recording**

    Close the previous terminal and in a new terminal, run the following command.

    ```sh
    ros2 launch walker walk_launch.py bag_record:=True
    ```
