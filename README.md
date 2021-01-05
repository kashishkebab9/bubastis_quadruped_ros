# bubastis_quadruped_ros
## About

My Quadruped Robot, Bubastis, simulated on ROS with Jacobian Kinematics.

There are two repositories for this project. This one and one named bubastis_quadruped_support. This is due to my lack of knowledge of github, oh well.

This repository contains all the code I wrote to ensure my math for the kinematics of a mammalian quadruped robot is correct. If you are interested in only looking at the math by referencing src/xbox_control/src/bubastis_backplane.py

## Pre-Requisites
1. A machine running Ubuntu. Only tested at this point on Ubuntu 16.04.06 LTS (Xenial)
2. ROS Kinetic installed

## Directions for Install  
1. If you don't have a catkin workspace you'll have to generate one:

        cd ~
        mkdir -p catkin_ws/src
        
2. Check out the project into the src directory:        
        
        cd ~/catkin_ws/src
        git clone https://github.com/kashishkebab9/bubastis_quadruped_ros.git
        
3. Install the dependencies for this project:
        
        cd ~/catkin_ws
        rosdep install --from-paths src -i -y
        
4. Build and source your workspace:
        
        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash
        
These instructions have been brought to you by user ahendrix (https://answers.ros.org/question/230798/sourcing-exsisting-ros-project-running-someone-elses-project/)


        
        

