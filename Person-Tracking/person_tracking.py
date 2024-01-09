#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64MultiArray
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import Marker

sensitivity_percentage = 80
startup_distance_requirement = 1
boundary_threshold = 0.8
max_detection_distance = 1

class Person_tracking:
    def __init__(self):
        pub_topic = "/person_lock_on"
        sub_topic = "/people_tracker_measurements"
        marker_topic = "/person_markers"

        self.pub = rospy.Publisher(pub_topic, Float64MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, PositionMeasurementArray, self.tracker_callback)

        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=10)
        self.frame = PositionMeasurementArray()
        self.tracker = list()

    def euclidean_distance(self, point1, point2):
        return math.sqrt(math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))


    def tracker_callback(self, msg):
        person_detected = len(msg.people)
        person_coord_list = list()

        print("Total people detected:", person_detected)
        if not self.tracker:
            if person_detected == 0:
                pass
            elif person_detected > 0:
                for i in range(person_detected):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                self.tracker = self.check_closest_point(coord_list=person_coord_list)
                if self.tracker is None:
                    print("Tracker initialization failed, skipping this frame")
                    pass
                elif self.tracker:
                    person_coord_list = list()
                    location_data = Float64MultiArray()
                    location_data.data = [self.tracker[0], self.tracker[1]]
                    self.pub.publish(location_data)
                    print("Tracker set on coordinate:", location_data.data)
                    self.publish_marker(self.tracker[0], self.tracker[1])

        elif self.tracker:
            if person_detected == 0:
                print("No person detected!")
                pass
            elif person_detected > 0:
                for i in range(person_detected):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                selected_person = self.check_lowest_offset(tracker=self.tracker, coord_list=person_coord_list)
                if selected_person is None:
                    print("Lost tracker as a person is moving too fast or out of range")
                    pass
                elif selected_person:
                    self.tracker = selected_person
                    person_coord_list = list()
                    location_data = Float64MultiArray()
                    location_data.data = [self.tracker[0], self.tracker[1]]
                    self.pub.publish(location_data)
                    print("Publish current coordinateeee:", location_data.data)
                    self.publish_marker(self.tracker[0], self.tracker[1])


    def check_closest_point(self, coord_list):
        score_list = list()

        for coord in range(len(coord_list)):
            score = abs(coord_list[coord][0]) + abs(coord_list[coord][1])
            score_list.append(score)

        selected_point_index = score_list.index(min(score_list))
        selected_coord = coord_list[selected_point_index]
        selected_coord_x = selected_coord[0]
        selected_coord_y = selected_coord[1]

        if abs(selected_coord_x) <= startup_distance_requirement and abs(selected_coord_y) < startup_distance_requirement:
            print("Track person coordinate:", selected_coord_x, selected_coord_y)
            return selected_coord
        else:
            print("People detected but unable to initialize tracker, please stand closer to the robot.")
            return None
        

    def check_lowest_offset(self, tracker, coord_list):
        tracker_range = 0.5
        score_list = list()

        for coord in range(len(coord_list)):
            euclidean_distance = self.euclidean_distance(point1=tracker, point2=coord_list[coord])
            score_list.append(euclidean_distance)

        smallest_distance = min(score_list)
        smallest_distance_index = score_list.index(smallest_distance)
        tracked_coord = coord_list[smallest_distance_index]

        print("Euclidean distance: ", smallest_distance)

        if smallest_distance < tracker_range:
            print("Track person coordinate: ", tracked_coord)
            return tracked_coord
        else:
            print("Lost tracker due to Euclidean distance limits")
            return None
        

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "/base_link"  # Change the frame_id if needed
        marker.header.stamp = rospy.Time.now()
        marker.ns = "person_markers"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('person_tracking')
    Person_tracking()
    rospy.spin()