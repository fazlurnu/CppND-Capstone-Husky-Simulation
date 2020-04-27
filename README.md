# CppND-Capstone-Husky-Simulation
A simulation of Husky using ROS and Gazebo for Udacity C++ Nanodegree Capstone Project

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
  * There is a compatibility issue with python3, so one has to use python2 to run this project. To install, on your terminal, type 'sudo apt install python2'
  
## Ubuntu - setup ROS & Gazebo:
This project has been tested on ROS Kinetic and Gazebo 7.0 on Ubuntu 16.04. [Click here for installation instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu). Select the Desktop-Full Install Repository.

For the simulation, a Husky package is required [Click here for the installation instructions](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./SnakeGame`.

