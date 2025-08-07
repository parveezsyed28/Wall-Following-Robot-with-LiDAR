**Wall-Following Robot with LiDAR**

**Overview**

This project implements a Wall-Following Robot equipped with a 2D LiDAR sensor, simulated in Gazebo and controlled using ROS 2. The robot autonomously navigates a maze-like environment by maintaining a fixed distance to the wall, using real-time laser scan data to avoid obstacles and follow the wall contour.

The robot model is designed in URDF/Xacro with realistic physics and sensor plugins. The control node is written in Python (ROS 2 rclpy) and publishes velocity commands based on LiDAR feedback.

**Features**

   --> Differential-drive robot simulated with Gazebo and ROS 2

   --> Accurate LiDAR simulation with Gazebo sensor plugin

   --> Real-time wall-following behavior using proportional control

   --> Launch files for seamless simulation setup (robot spawn, world load, nodes)

   --> Fully modular ROS 2 package structure

   --> Easy to extend with custom worlds or control algorithms

**Project Structure**
```
wall_follower_bot/
├── launch/
│   └── state_publisher.launch.py # Launch Gazebo and spawn robot
├── models/
│   ├── meshes/                   # STL files for robot parts
│   ├── urdf/
│   │   ├── wall_bot.xacro        # Main robot description
│   │   ├── wall_bot.gazebo       # Gazebo-specific plugin configs
│   │   └── materials.xacro       # Material definitions
├── wall_follower_bot/
│   └── wall_follower_bot_node.py # Wall follower control node (Python)
├── worlds/
│   └── big_maze.world            # Maze environment for Gazebo
├── package.xml                  # ROS 2 package config
├── setup.py                    # Python package setup
└── README.md                   # This file
```

**Prerequisites**

    Ubuntu 22.04

    ROS 2 Humble

    Gazebo simulator (comes with ROS 2 desktop full installation)

    xacro package installed:

    sudo apt install ros-humble-xacro

    Python 3 with ROS 2 rclpy

**Installation & Setup**

**Clone the repository into your ROS 2 workspace source folder**
'''
cd ~/ros2_ws/src
git clone https://github.com/parveezsyed28/Wall-Following-Robot-with-LiDAR.git wall_follower_bot
'''

**Build the workspace**
'''
    cd ~/ros2_ws
    colcon build --packages-select wall_follower_bot
    source install/setup.bash
'''

**Running the Simulation**

Launch the Gazebo world with the robot and wall follower node:
'''
ros2 launch wall_follower_bot gazebo.launch.py
'''
This will:

    Start Gazebo with the maze environment (big_maze.world)

    Spawn the robot model in Gazebo using the processed URDF/Xacro

    Launch the robot_state_publisher to publish TFs from the URDF

    Run the wall follower control node that subscribes to LiDAR data and publishes velocity commands to drive the robot

**How It Works**

    The robot uses a differential drive plugin for Gazebo to simulate wheel control.

    The LiDAR plugin simulates a 2D laser scanner providing real-time distance measurements around the robot.

    The wall follower node listens on the /ttb_lidar/out topic for sensor_msgs/LaserScan messages.

    It calculates the distance to the right wall (approx. -90 degrees) and front.

    Using a simple proportional controller, it adjusts angular velocity to maintain a desired distance (0.5 m) from the right wall.

    If an obstacle is detected in front (less than 0.3 m), the robot turns left to avoid collision.

**Code Highlights**

**Wall Follower Node (Python):**

    Subscribes to LiDAR scan topic /ttb_lidar/out.

    Extracts right and front distances from laser data.

    Publishes velocity commands (geometry_msgs/Twist) to cmd_vel to drive the robot.

**URDF/Xacro Robot Model:**

    Describes robot links, joints, and visual/collision meshes.

    Includes Gazebo plugins for differential drive and LiDAR simulation.

    Configured for ROS 2 and Gazebo integration.

**Launch Files:**

    gazebo.launch.py: Launches Gazebo with maze world and spawns the robot.

    Declares use_sim_time argument to synchronize simulation time.

**Troubleshooting**

    1)If the robot is not moving despite publishing velocity commands, ensure:

       - The differential drive plugin in URDF is correctly configured with proper joint names.

       - The cmd_vel topic matches the topic your robot's controller listens to.

       - Gazebo physics is enabled and no errors appear in the terminal.

   2) For teleoperation, ensure the teleop node publishes to the correct cmd_vel topic.

    3)Use ros2 topic echo /cmd_vel to verify velocity commands.

**Future Improvements**

    - Add PID controller for smoother wall following.

    - Integrate SLAM for mapping unknown environments.

    - Implement dynamic obstacle avoidance.

    - Extend to multi-floor maze environments.
