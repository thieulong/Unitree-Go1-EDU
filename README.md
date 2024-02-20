# Unitree-Go1-EDU Project 

## Description 
<img width="310" alt="Diagram-of-overall-system" src="https://github.com/thieulong/Unitree-Go1-EDU/assets/53591284/1fe15416-d4b1-4d6f-879c-460af740315d">


This project compares the performance of differential-drive and omni-drive robots in Human-Robot Interaction (HRI) scenarios, specifically focusing on robot following behaviors. 
It explores various aspects such as following strategies, obstacle avoidance techniques, and robot positioning. 
The objective is to gain insights into different facets of HRI, including robot positioning, alternative path-planning algorithms, transitions between following and guiding, and the impact of additional tasks like carrying loads.

The proposed solution utilizes ROS Melodic for software infrastructure, 2D Lidar for person tracking, and integrated ultrasonic sensors for obstacle avoidance on a quadruped robot platform. 
The aim is to develop a robust system capable of stable tracking in diverse environments.

## Installation

This project is using ROS 1 with Ubuntu 18.04 lts
To install Ubuntu 18 ROS Melodic: https://wiki.ros.org/melodic/Installation/Ubuntu

From ROS Melodic we need extra packages:
1. Easy makers: https://wiki.ros.org/easy_markers
2. Kalman filter: https://wiki.ros.org/robot_pose_ekf
3. Movebase: https://wiki.ros.org/move_base
4. Leg detecter: https://wiki.ros.org/leg_detector

From the dog we need to add the robot's ROS_MASTER_URI to bash.rc:
1. `cd ~`
2. `nano .bashrc`
3. scroll to the bottom of the file and add these line:


To set up the unitree Go1: https://m.unitree.com/download/go1/

To build the environment:
1. `mkdir ~/catkin_ws/src`
2. `cd ~/catkin_ws/src`
3. `git clone https://github.com/thieulong/Unitree-Go1-EDU.git`
4. `cd ~/catkin_ws`
5. `catkin_make`
6. `source devel/setup.bash`

## Usage

### Dog Following
***Those command needs to be run concurrently in separate terminals***
1. `roslaunch hotdog_person_follow detect_track_follow.launch`
2. `roslaunch unitree_legged_real avoid_and_follow.launch`

### Change Dog Following Position/Operation Mode

- Positions:
1. Behind: `rosparam set /following_position 1`
1. Side by side: `rosparam set /following position 2`

- Operation modes:
1. Differential: `rosparam set /operation_mode 1`
2. Omni: `rosparam set /operation_mode 2`
3. Spin and go: `rosparam set /operation_mode 3`


## Configuration

1. The robot dog stays in the `/high_cmd`
2. The robot dog positions and operation modes table


| Operation modes \ Positions | Pos: 1 | Pos: 2 |
| :- | :---------------: | :---------------: |
| Modes | Following at back  | Following side by side |
| 1 | Diff | Diff |
| 2 | Omni | Omni |
| 3 | Spin&Go | Spin&Go |

3. The Lidar may need a back up powerbank for working consistency 

### For people working on the current dog
1. Go342 cannot use `/cmd_vel`, we have to use `/high_cmd` from unitree
2. The dog might need to move around a bit for the Wi-Fi connection

## Contributing

Leave a message to join if interested in contributing to this project
