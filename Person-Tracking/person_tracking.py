#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from people_msgs.msg import PositionMeasurementArray

sensitivity_percentage = 80
startup_distance_requirement = 50
boundary_threshold = 1.8
max_detection_distance = 2

class Person_tracking:
    def __init__(self):
        pub_topic = "/person_lock_on"
        sub_topic = "/people_tracker_measurements"

        # self.pub = rospy.Publisher(pub_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, PositionMeasurementArray, self.tracker_callback)

    def check_closest_point(self, coord_list):
        score_list = list()
        startup_distance_requirement = startup_distance_requirement/100
        for coord in range(len(coord_list)):
            score_list.append(coord_list[coord][0] + coord_list[coord][1])

        selected_point_index = score_list.index(min(score_list))
        if abs(coord_list[selected_point_index][0]) < startup_distance_requirement and abs(coord_list[selected_point_index][1]) < startup_distance_requirement:
            rospy.loginfo(f"Lock-on person coordinate: {coord_list[selected_point_index][0], coord_list[selected_point_index][1]}")
            return selected_point_index
        else:
            return None
        
        
    def check_lowest_offset(self, prev_frame, coord_list):
        prev_frame_x = prev_frame[0]
        prev_frame_y = prev_frame[1]
        sensitivity_scale = (100-sensitivity_percentage)/100
        
        for coord in range(len(coord_list)):
            coord_x = coord_list[coord][1]
            coord_y = coord_list[coord][1]
            if coord_x in range(prev_frame_x-sensitivity_scale, prev_frame_x+sensitivity_scale) and coord_y in range(prev_frame_y-sensitivity_scale, prev_frame_y+sensitivity_scale):
                curr_frame = coord_list[coord]
                return curr_frame
            else:
                return None
                   
            
    def tracker_callback(self, msg):
        # print(msg.people)

        person_coord_list = list()
        prev_frame = list()
        curr_frame = list()

        rospy.loginfo(f"Total people detected: {len(msg.people)}")
        if not prev_frame:
            if len(msg.people) == 0:
                pass
            elif len(msg.people) > 0:
                for i in range(len(msg.people)):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                closest_person_index = self.check_closest_point(coord_list=person_coord_list)
                selected_person = person_coord_list[closest_person_index]
                prev_frame = selected_person
                person_coord_list.clear()
                return prev_frame

        elif prev_frame:
            if len(msg.people) == 0:
                pass
            elif len(msg.people) > 0:
                for i in range(len(msg.people)):
                    person_coord_list.append([msg.people[i].pos.x, msg.people[i].pos.y])
                selected_person = self.check_lowest_offset(prev_frame=prev_frame, coord_list=person_coord_list)
                curr_frame = selected_person
                prev_frame = curr_frame
                person_coord_list.clear()
                curr_frame.clear()
                return prev_frame
                
if __name__ == '__main__':
    rospy.init_node('person_tracking')
    Person_tracking()
    rospy.spin()    