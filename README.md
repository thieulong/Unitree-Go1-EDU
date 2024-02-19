# Unitree-Go1-EDU Project 

## Description 
<img width="310" alt="Diagram-of-overall-system" src="https://github.com/thieulong/Unitree-Go1-EDU/assets/53591284/1fe15416-d4b1-4d6f-879c-460af740315d">


This project compares the performance of differential-drive and omni-drive robots in Human-Robot Interaction (HRI) scenarios, specifically focusing on robot following behaviors. 
It explores various aspects such as following strategies, obstacle avoidance techniques, and robot positioning. 
The objective is to gain insights into different facets of HRI, including robot positioning, alternative path-planning algorithms, transitions between following and guiding, and the impact of additional tasks like carrying loads.

The proposed solution utilizes ROS Melodic for software infrastructure, 2D Lidar for person tracking, and integrated ultrasonic sensors for obstacle avoidance on a quadruped robot platform. 
The aim is to develop a robust system capable of stable tracking in diverse environments.

## Installation

This project is using ROS 1 with Ubuntu 18.
To install Ubuntu 18 ROS Melodic: https://wiki.ros.org/melodic/Installation/Ubuntu

From ROS Melodic we need extra packages:
1. Easy makers: https://wiki.ros.org/easy_markers
2. Kalman filter: https://wiki.ros.org/robot_pose_ekf
3. Movebase: https://wiki.ros.org/move_base
4. Leg detecter: https://wiki.ros.org/leg_detector

## Usage

### Dog Following
***Those command needs to be run concurrently in separate terminals***
1. `roslaunch hotdog_person_follow detect_track_follow.launch`
2. `roslaunch unitree_legged_real avoid_and_follow.launch`


## Configuration

1. The robot dog stays in the /high_cmd
2. The robot dog positions and operation modes table
| Operation modes \ Positions | :-: | -: |
| :- | :---------------: | ---------------: |
| :- | Following at back (Pos:1)  | Side by side (Pos:2) |
| 1 | Diff | Diff |
| 2 | Omni | Omni |
| 3 | Spin2Go | Spin2Go |

## Contributing

Leave a message to join if interested in contributing this project

## License
...
