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

        self.obstacle_topic = "/obstacle_zone"
        self.obstacle_pub = rospy.Publisher(self.obstacle_topic, Float64MultiArray, queue_size=1)

        self.front_limit = 0.2
        self.back_limit = 0.23
        self.left_limit = 0.15
        self.right_limit = 0.15

        self.tracker_x = 0
        self.tracker_y = 0
        self.tracker_x_prev = 0
        self.tracker_y_prev = 0
        self.tracker_zone = None
        self.tracker_angle = 0

        self.person_angle = 0
        self.angle_thres = 5

    def calc_mean(self, values):
        filtered_list = [v for v in values if not math.isnan(v) and not math.isinf(v)]
        return sum(filtered_list) / len(filtered_list) if filtered_list else 0
    
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

    def lidar_callback(self, msg):

        self.obstacle_zones = list()

        tracking_coords = [self.tracker_x, self.tracker_y]
        tracking_angle = self.calc_angle(tracker_coords=tracking_coords)
        tracking_angle_lower = tracking_angle - self.angle_thres
        tracking_angle_upper = tracking_angle + self.angle_thres

        if tracking_angle_lower < 0: tracking_angle_lower = 0
        if tracking_angle_upper > 360: tracking_angle_upper = 360

        distance_list = list(msg.ranges)
        distance_list[tracking_angle_lower:tracking_angle_upper] = [numpy.nan] * len(range(tracking_angle_lower, tracking_angle_upper))

        zone_1 = distance_list[0:45]
        zone_2 = distance_list[45:90]
        zone_3 = distance_list[90:135]
        zone_4 = distance_list[135:180]
        zone_5 = distance_list[180:225]
        zone_6 = distance_list[225:270]
        zone_7 = distance_list[270:315]
        zone_8 = distance_list[315:360]

        zone_1_mean = self.calc_mean(values=zone_1)
        zone_2_mean = self.calc_mean(values=zone_2)
        zone_3_mean = self.calc_mean(values=zone_3)
        zone_4_mean = self.calc_mean(values=zone_4)
        zone_5_mean = self.calc_mean(values=zone_5)
        zone_6_mean = self.calc_mean(values=zone_6)
        zone_7_mean = self.calc_mean(values=zone_7)
        zone_8_mean = self.calc_mean(values=zone_8)

        if zone_1_mean <= self.back_limit: self.obstacle_zones.append(1)
        if zone_2_mean <= self.left_limit: self.obstacle_zones.append(2)
        if zone_3_mean <= self.left_limit: self.obstacle_zones.append(3)
        if zone_4_mean <= self.front_limit: self.obstacle_zones.append(4)
        if zone_5_mean <= self.front_limit: self.obstacle_zones.append(5)
        if zone_6_mean <= self.right_limit: self.obstacle_zones.append(6)
        if zone_7_mean <= self.right_limit: self.obstacle_zones.append(7)
        if zone_8_mean <= self.back_limit: self.obstacle_zones.append(8)
        else: self.obstacle_zones.append(0)

        combined_data = Float64MultiArray()
        combined_data.data = [self.tracker_x_prev, self.tracker_y_prev, self.tracker_x, self.tracker_y, self.obstacle_zones]
        self.obstacle_pub.publish(combined_data)

if __name__ == '__main__':
    rospy.init_node('obstacle_detection')
    DetectObstacles
    rospy.spin()