# CppND-Capstone-Husky-Simulation
A simulation of Husky using ROS and Gazebo for Udacity C++ Nanodegree Capstone Project. The project I choose is a ROS Node simulation in Gazebo. This project is interesting for me because I learn C++ to advance my skill in robotics.

The idea of this project is from a course at [ETH Zurich](https://rsl.ethz.ch/education-students/lectures/ros.html), exercise 3. The goal is to have a closed-loop control of the Husky. Husky reads position of a pillar using lidar and drives towards the pillar automatically. More detail on the exercise can be found [here](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2020/Exercise%20Session%203.pdf). One can also have the position of Husky and pillar visualized in Rviz.

## Rubric Points
* Readme (All Rubric Points)

* Compiling and Testing (All Rubric Points)

* Loops, Functions, I/O:
   * A variety of control structures are used in the project (see [HuskyHighlevelController.cpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/src/husky_highlevel_controller_node.cpp) line 60)
   * The project code is clearly organized into functions (see [HuskyHighlevelController.cpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/src/husky_highlevel_controller_node.cpp))
   * The project reads data from an external file (see [HuskyHighlevelController.cpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/src/husky_highlevel_controller_node.cpp) line 27). The use of param file is defined in [husky_single_pillar.launch](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/launch/husky_single_pillar.launch) line 19. The param file is [here](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/tree/master/param) 

* Object Oriented Programming:
    * The project code is organized into classes with class attributes hold the data, and class methods to perform task (see [HuskyHighlevelController.hpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/include/husky_highlevel_controller/HuskyHighlevelController.hpp)
    * All class data members are explicitly specified as public, protected, or priveate (see [HuskyHighlevelController.hpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/include/husky_highlevel_controller/HuskyHighlevelController.hpp)
    * All class members that are set to argument values are initialized through member initialization lists (see [HuskyHighlevelController.cpp](https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/src/husky_highlevel_controller_node.cpp) line 6)

* Memory Management:
    * At least two functions use pass-by-reference in the project code ((https://github.com/fazlurnu/CppND-Capstone-Husky-Simulation/blob/master/src/husky_highlevel_controller_node.cpp) line 5 and line 49)
    

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* catkin_tools = latest version
  * Linux: [Click here for installation instructions](https://catkin-tools.readthedocs.io/en/latest/installing.html)
* python2
  * There is a compatibility issue with python3, so one has to use python2 to run this project. To install, on your terminal, type `sudo apt install python2`
  
## Ubuntu - setup ROS & Gazebo:
This project has been tested on ROS Kinetic and Gazebo 7.0 on Ubuntu 16.04. [Click here for installation instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu). Select the Desktop-Full Install Repository.

For the simulation, a Husky package is required [Click here for the installation instructions](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./SnakeGame`.
