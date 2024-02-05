#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import math

class LidarDistance:
    def __init__(self):
        self.lidar_topic = "/scan"
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)
        self.tracker_topic = "/person_tracking"
        self.tracker_sub = rospy.Subscriber(self.tracker_topic, Float64MultiArray, self.tracker_callback)

        self.pub = rospy.Publisher('/avoid_obstacles', Float64MultiArray, queue_size=1)

        self.front_limit = 0.2
        self.back_limit = 0.23
        self.left_limit = 0.15
        self.right_limit = 0.15

        self.front_lower_range = 135
        self.front_upper_range = 225
        
        self.back_lower_range = 45
        self.back_upper_range = 315
        
        self.left_lower_range = 45
        self.left_upper_range = 135

        self.right_lower_range = 225
        self.right_upper_range = 315

        self.tracker_x = 0
        self.tracker_y = 0
        self.tracker_x_prev = 0
        self.tracker_y_prev = 0
        self.tracker_zone = None
        self.tracker_angle = 0

        self.person_angle = 0
        self.angle_thres = 5
        self.angle_extra = 5

        self.obstacle_direction = -2

    def calc_mean(self, values):
        filtered_list = [v for v in values if not math.isnan(v) and not math.isinf(v)]
        return sum(filtered_list) / len(filtered_list) if filtered_list else 0
    
    def calc_angle(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        angle = abs(math.degrees(math.atan(x/y)))
        angle = round(angle)
        return angle
    
    def assign_tracker_zone(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        if x > 0 and y > 0:
            tracker_zone = "TR"
        elif x > 0 and y < 0:
            tracker_zone = "BR"
        elif x < 0 and y > 0:
            tracker_zone = "TL"
        elif x < 0 and y < 0:
            tracker_zone = "BL"
        return tracker_zone
    
    def tracker_callback(self, msg):
        self.tracker_x_prev = msg.data[0]
        self.tracker_y_prev = msg.data[1]
        self.tracker_x = msg.data[2]
        self.tracker_y = msg.data[3]

    def lidar_callback(self, msg):
        tracker_coords = [self.tracker_x, self.tracker_y]
        self.tracker_zone = self.assign_tracker_zone(tracker_coords=tracker_coords)
        self.tracker_angle = self.calc_angle(tracker_coords=tracker_coords)

        if self.tracker_zone == "TR":
            self.person_angle = 180 + (90 - self.tracker_angle)
        elif self.tracker_zone == "BR":
            self.person_angle = 270 + self.tracker_angle
        elif self.tracker_zone == "TL":
            self.person_angle = 90 + self.tracker_angle
        elif self.tracker_zone == "BL":
            self.person_angle = self.tracker_angle

        distance_list = list(msg.ranges)

        if self.person_angle in range(self.front_lower_range, self.front_upper_range):
            lower_bound = self.person_angle - self.angle_thres
            upper_bound = self.person_angle + self.angle_thres
            if lower_bound < self.front_lower_range: 
                upper_bound += self.angle_extra
                front = distance_list[upper_bound:self.front_upper_range]
            elif upper_bound > self.front_upper_range: 
                lower_bound -= self.angle_extra
                front = distance_list[self.front_lower_range:lower_bound]
            else:
                front = distance_list[self.front_lower_bound:lower_bound] + distance_list[upper_bound:self.front_upper_range]

        if self.person_angle in range(self.left_lower_range, self.left_upper_range):
            lower_bound = self.person_angle - self.angle_thres
            upper_bound = self.person_angle + self.angle_thres
            if lower_bound < self.left_lower_range: 
                upper_bound += self.angle_extra
                left = distance_list[upper_bound:self.left_upper_range]
            elif upper_bound > self.left_upper_range: 
                lower_bound -= self.angle_extra
                left = distance_list[self.left_lower_range:lower_bound]
            else:
                left = distance_list[self.left_lower_bound:lower_bound] + distance_list[upper_bound:self.left_upper_range]      

        if self.person_angle in range(self.right_lower_range, self.right_upper_range):
            lower_bound = self.person_angle - self.angle_thres
            upper_bound = self.person_angle + self.angle_thres
            if lower_bound < self.right_lower_range: 
                upper_bound += self.angle_extra
                right = distance_list[upper_bound:self.right_upper_range]
            elif upper_bound > self.right_upper_range: 
                lower_bound -= self.angle_extra
                right = distance_list[self.right_lower_range:lower_bound]
            else:
                right = distance_list[self.right_lower_bound:lower_bound] + distance_list[upper_bound:self.right_upper_range] 

        back = distance_list[:self.back_lower_range] + distance_list[self.back_upper_range:]
        if self.person_angle in back:
            lower_bound = self.person_angle - self.angle_thres
            upper_bound = self.person_angle + self.angle_thres
            if lower_bound < 0:
                lower_bound = 360 - self.angle_thres
                back = distance_list[self.back_lower_range:lower_bound] + distance_list[upper_bound:self.back_upper_range]
            elif upper_bound > 360:
                upper_bound = self.angle_thres
                back = distance_list[self.back_lower_range:lower_bound] + distance_list[upper_bound:self.back_upper_range]
            elif self.person_angle < self.back_lower_range:
                back = distance_list[:lower_bound] + distance_list[upper_bound:self.back_lower_range] + distance_list[self.back_upper_range:]
            elif self.person_angle > self.back_upper_range:
                back = distance_list[:self.back_lower_range] + distance_list[self.back_upper_range:lower_bound] + distance_list[upper_bound:]


        front_current = self.calc_mean(front)
        back_current = self.calc_mean(back)
        left_current = self.calc_mean(left)
        right_current = self.calc_mean(right)
        
        if front_current == 0 or back_current == 0 or left_current == 0 or right_current == 0:
            self.obstacle_direction = -2
        if front_current < self.front_limit:
            self.obstacle_direction = 0
        elif back_current < self.back_limit:
            self.obstacle_direction = 1
        elif left_current < self.left_limit:
            self.obstacle_direction = 2
        elif right_current < self.right_limit:
            self.obstacle_direction = 3
        else:
            self.obstacle_direction = -1

        obstacle_data = Float64MultiArray()
        obstacle_data.data = [self.tracker_x_prev, self.tracker_y_prev, self.tracker_x, self.tracker_y, self.obstacle_direction]
        self.pub.publish(obstacle_data)

if __name__ == '__main__':
    rospy.init_node('lidar_distance')
    LidarDistance()
    rospy.spin()
