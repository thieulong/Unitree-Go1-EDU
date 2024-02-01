#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import math
import numpy

class DetectObstacles:
    def __init__(self):

        self.lidar_topic = "/scan"
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)

        self.tracker_topic = "/person_tracking"
        self.tracker_sub = rospy.Subscriber(self.tracker_topic, Float64MultiArray, self.tracker_callback)

        self.obstacle_topic = "/obstacle_coordinates"
        self.obstacle_pub = rospy.Publisher(self.obstacle_topic, Float64MultiArray, queue_size=1)

        self.distance_limit = 0.2

        self.tracker_x = 0
        self.tracker_y = 0
        self.tracker_x_prev = 0
        self.tracker_y_prev = 0
        self.tracker_angle = 0

        self.person_angle = 0
        self.angle_thres = 5

        self.obstacle_size = 10

    def calc_angle(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]

        angle = abs(math.degrees(math.atan(x/y)))
        angle = round(angle)
        if x < 0 and y < 0:
            self.tracker_angle = angle
        elif x < 0 and y > 0:
            self.tracker_angle = 90 + (90 - angle)
        elif x > 0 and y > 0:
            self.tracker_angle = 180 + angle
        elif x > 0 and y < 0:
            self.tracker_angle = 270 + (90 - angle)
        
        return self.tracker_angle
    
    def tracker_callback(self, msg):
        self.tracker_x_prev = msg.data[0]
        self.tracker_y_prev = msg.data[1]
        self.tracker_x = msg.data[2]
        self.tracker_y = msg.data[3]

    def calc_coord(self, dist, angle):
        x_coord = dist * math.sin(angle)
        y_coord = dist * math.cos(angle)
        if angle >= 0 and angle < 90:
            x_coord = x_coord * -1
            y_coord = y_coord * -1
        elif angle >= 90 and angle < 180:
            x_coord = x_coord * -1
            y_coord = y_coord * 1
        elif angle >= 180 and angle < 270:
            x_coord = x_coord * 1
            y_coord = y_coord * 1
        elif angle >= 270 and angle <=360:
            x_coord = x_coord * 1
            y_coord = y_coord * -1

        return [x_coord, y_coord]

    def detect_angle_sequence(self, angles_list):
        angle_sequence = list()
        current_sequence = list()

        for i, (a, b) in enumerate(zip(angles_list, angles_list[1:])):
            if b - a == 1:
                current_sequence.append(b)
            else:
                if len(current_sequence) > self.obstacle_size:
                    angle_sequence.append(current_sequence)
                current_sequence = list()
        if len(current_sequence) > self.obstacle_size:
            angle_sequence.append(current_sequence)

        if not angle_sequence:
            print("Obstacle detected in range but the area is smaller than desired size, skipping it in this frame.")
        else:
            return angle_sequence

    def lidar_callback(self, msg):

        self.obstacle_coords = list()
        self.obstacle_angles = list()
        self.sequence_angles = list()
        self.angles_start_end = list()

        tracking_coords = [self.tracker_x, self.tracker_y]
        tracking_angle = self.calc_angle(tracker_coords=tracking_coords)
        tracking_angle_lower = tracking_angle - self.angle_thres
        tracking_angle_upper = tracking_angle + self.angle_thres

        if tracking_angle_lower < 0: tracking_angle_lower = 0
        if tracking_angle_upper > 360: tracking_angle_upper = 360

        distance_list = list(msg.ranges)
        distance_list[tracking_angle_lower:tracking_angle_upper] = [numpy.nan] * len(range(tracking_angle_lower, tracking_angle_upper))

        for i in range(len(distance_list)):
            if distance_list[i] < self.distance_limit:
                self.obstacle_angles.append(i)

        self.sequence_angles = self.detect_angle_sequence(angles_list=self.obstacle_angles)

        if self.sequence_angles:
            total_obstacles = len(self.sequence_angles)
            print("Number of obstacles:", total_obstacles)
            for i in range(total_obstacles):
                self.obstacle_angles.append([self.sequence_angles[i][0], self.sequence_angles[i][-1]])
            
        for angle in self.obstacle_angles:
            angle_start = angle[0]
            angle_end = angle[-1]
            dist_start = distance_list[angle_start]
            dist_end = distance_list[angle_end]
            coord_start = self.calc_coord(dist=dist_start, angle=angle_start)
            coord_end = self.calc_coord(dist=dist_end, angle=angle_end)
            obstacle_coords = [coord_start, coord_end]
            joined_obstacle_coords = [item for sublist in obstacle_coords for item in sublist]
            print("Obstacles in frame:", joined_obstacle_coords)
            self.obstacle_coords.append(joined_obstacle_coords)

        combined_data = Float64MultiArray()
        combined_data.data = [self.tracker_x_prev, self.tracker_y_prev, self.tracker_x, self.tracker_y, self.obstacle_coords]
        self.obstacle_pub.publish(combined_data)

if __name__ == '__main__':
    rospy.init_node('obstacle_detection')
    DetectObstacles
    rospy.spin()


