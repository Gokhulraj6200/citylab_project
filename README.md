# Autonomous Obstacle Avoidance with Turtlebot3
This project implements Obstacle Avoidance algorithm using ROS2 and c++. The code is tested in both simulation and real Turtlebot3 setup.
 
## Table of Contents

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#result)
- [Contributing](#contributing)

## Introduction <a name="introduction"></a>
The main objective of this project is to achieve a precise motion for the turtlebot3 in a given setup. The project encapulates basic ROS2 services and actions to control a Turtlebot3 robot for autonomous patrolling and precise navigation to designated positions.  The robot uses Laserscan to identify the nearest obstacle, which then decide and navigate to the safest calculated distance. 

## Requirements <a name="requirements"></a>

- ROS2 Humble
- Gazebo
- c++ compiler
- Linux
- Turtlebot3 simulation package or a real Turtlebot3 robot
  
## Installation <a name="installation"></a>

1\. clone this repository inside your `catkin_ws/src` directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Gokhulraj6200/citylab_project.git
```

2\. Compile the package:
```bash
cd ~/ros2_ws
colcon build
```

3\. Source the ROS environment:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage <a name="usage"></a>
1\. Using the following command, source and start the Turtlebot3 simulation: 
```bash
source ~/simulation_ws/install/setup.bash
ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml
```

2\. In another terminal, launch the robotpatrol file: 
```bash
ros2 launch robot_patrol start_patrolling.launch.py   
```

## Results <a name="result"></a>
The Turtlebot3 takes multiple laps inside the environment:



https://github.com/Gokhulraj6200/citylab_project/assets/142083650/035e848a-b61f-49ac-bd80-726ee03fe274



## Contributing
Feel free to contribute to this project by following these simple steps:

1\. Fork the repository.

2\. Create a new branch for your input: `git checkout -b branch-name`.

3\. Make your changes and commit them: `git commit -m 'Add new feature'`.

4\. Push to the branch: `git push origin branch-name`.
