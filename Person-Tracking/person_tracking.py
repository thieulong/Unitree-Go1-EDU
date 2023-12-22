#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64MultiArray
from people_msgs.msg import PositionMeasurementArray

sensitivity_percentage = 80
startup_distance_requirement = 1
boundary_threshold = 0.8
max_detection_distance = 1

class Person_tracking:
    def __init__(self):
        pub_topic = "/person_lock_on"
        sub_topic = "/people_tracker_measurements"

        self.pub = rospy.Publisher(pub_topic, Float64MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, PositionMeasurementArray, self.tracker_callback)
        self.frame = PositionMeasurementArray()

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

        tracker_range = math.sqrt(2)
        score_list = list()
        
        for coord in range(len(coord_list)):
            euclidean_distance = math.dist(tracker, coord_list[coord])
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
                   
            
    def tracker_callback(self, msg):
        person_detected = len(msg.people)
        person_coord_list = list()

        tracker = list()
        frame = list()

        print("Total people detected:", person_detected)
        if not tracker:
            if person_detected == 0:
                pass
            elif person_detected > 0:
                for i in range(person_detected):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                tracker = self.check_closest_point(coord_list=person_coord_list)
                if tracker is None:
                    print("Tracker initialization failed, skipping this frame")
                    pass
                elif tracker:
                    person_coord_list = list()
                    location_data = Float64MultiArray()
                    location_data.data = [tracker[0], tracker[1]]
                    self.pub.publish(location_data)
                    print("Tracker set on coordinate:", location_data.data)

        elif tracker:
            if person_detected == 0:
                print("No person detected!")
                pass
            elif person_detected > 0:
                for i in range(person_detected):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                selected_person = self.check_lowest_offset(tracker=tracker, coord_list=person_coord_list)
                if selected_person is None:
                    print("Lost tracker as person is moving too fast or out of range")
                    pass
                elif selected_person:
                    frame = selected_person
                    tracker = frame
                    person_coord_list = list()
                    frame = list()
                    location_data = Float64MultiArray()
                    location_data.data = [tracker[0], tracker[1]]
                    self.pub.publish(location_data)
                    print("Publish current coordinateeee:", location_data.data)
                
if __name__ == '__main__':
    rospy.init_node('person_tracking')
    Person_tracking()
    rospy.spin()    