#!/usr/bin/env python
import rospy
thres_per = 0.2

class Movebased_Segment:
    def __init__(self):

    def move_callback(self, msg):
        rows = len(msg.data)
        cols = len(msg.data[0])
        num_sections = 3
        cut_1 = cols // num_sections
        cut_2 = 2*cut_1

        num_elements_in_section = rows*(cols//num_sections)

        # assume 0 is not ground
        section_1 = msg[:][:cut_1].count(0)/num_elements_in_section
        section_2 = msg[:][cut_1:cut_2].count(0)/num_elements_in_section
        section_3 = msg[:][cut_2:cols].count(0)/num_elements_in_section
        all = [section_1,section_2,section_3]
        smallest = all.index(min(all))

        if min(all) >= thres_per:
            # dont move 
        else:
            if smallest == 0:
                # turn left
            elif smallest == 1:
                # forward
            elif smallest == 2:
                # turn right
            else:
                print("Something wrong")
                pass

if __name__ == '__main__':
    rospy.init()
    Movebased_Segment()
    rospy.spin()




    
    
    
    
    
    
    
    
    
    
    
    
    





