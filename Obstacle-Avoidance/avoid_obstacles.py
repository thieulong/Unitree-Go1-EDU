#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32  
from sensor_msgs.msg import LaserScan
import math

class LidarDistance:
    def __init__(self):
        self.sub_topic = "/scan"
        self.sub = rospy.Subscriber(self.sub_topic, LaserScan, self.lidar_callback)

        self.pub = rospy.Publisher('/avoid_obstacles', Int32, queue_size=1)

        self.front_limit = 0.2
        self.back_limit = 0.2
        self.left_limit = 0.2
        self.right_limit = 0.2

    def calc_mean(self, values):
        filtered_list = [v for v in values if not math.isnan(v) and not math.isinf(v)]
        return sum(filtered_list) / len(filtered_list) if filtered_list else 0

    def lidar_callback(self, msg):
        distance_list = list(msg.ranges)

        front = distance_list[135:225]
        back = distance_list[:45] + distance_list[315:]
        left = distance_list[45:135]
        right = distance_list[225:315]

        front_current = self.calc_mean(front)
        back_current = self.calc_mean(back)
        left_current = self.calc_mean(left)
        right_current = self.calc_mean(right)

        if front_current == 0 or back_current == 0 or left_current == 0 or right_current == 0:
            self.publish_avoid_obstacles(-2)

        if front_current < self.front_limit:
            self.publish_avoid_obstacles(0)
        elif back_current < self.back_limit:
            self.publish_avoid_obstacles(1)
        elif left_current < self.left_limit:
            self.publish_avoid_obstacles(2)
        elif right_current < self.right_limit:
            self.publish_avoid_obstacles(3)
        else:
            self.publish_avoid_obstacles(-1)  

    def publish_avoid_obstacles(self, value):
        msg = Int32()
        msg.data = value
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('lidar_distance')
    LidarDistance()
    rospy.spin()
