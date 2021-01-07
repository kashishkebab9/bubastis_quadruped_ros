# bubastis_quadruped_ros
## About

My Quadruped Robot, Bubastis, simulated on ROS with Jacobian Kinematics.

There are two repositories for this project. This one and one named bubastis_quadruped_support. This is due to my lack of knowledge of github, oh well.

This repository contains all the code I wrote to ensure my math for the kinematics of a mammalian quadruped robot is correct. If you are interested in only looking at the math by referencing src/xbox_control/src/bubastis_backplane.py

## Pre-Requisites
1. A machine running Ubuntu. Only tested at this point on Ubuntu 16.04.06 LTS (Xenial)
2. ROS Kinetic installed
3. An Xbox 360 controller to interface with linux
4. Have the Joy Node set up in your ROS Distro: http://wiki.ros.org/joy
5. Python 2.7 needs to be installed, Python 3 will NOT work

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

# Directions for Use:

1. Open three Terminal Windows (T1, T2, T3)
2. In T1, source the project:
                
                
                cd ~/catkin_ws/src/bubaastis_quadruped_ros
                source devel/setup.bash
                
3. Launch the robot model in rviz:
                
                cd ~/catkin_ws/src/bubaastis_quadruped_ros/src
                roslaunch robot_model rviz.launch
                
4. Connect the xbox controller and get the controller 'jsx' (it will be js1, js2, etc. for you) configured in T2 with the following:
                
                
                ls dev/input
                sudo chmod a+rw /dev/input/jsX
                rosparam set joy_node/dev "/dev/input/jsX"
                rosrun joy joy_node
   4a. You can run the following and start pressing buttons on the controller to make sure communcation is working between your machine and xbox:

                rostopic echo /joy
                
5. In T3, run the following:

                cd ~/catkin_ws/src/bubaastis_quadruped_ros
                source devel/setup.bash
                cd src
                rosrun xbox_control jacobian_directional_walking_controller.py
                
6. Now you can move the left joystick in any direction, and while holding it there, press the right back trigger, Bubastis should move in that direction!                
               
        
        

