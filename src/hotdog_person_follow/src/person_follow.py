#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class Person_follow:
    def __init__(self):
        pub__topic = "/cmd_vel"
        sub__topic = "/person_tracking"

        self.pub_ = rospy.Publisher(pub__topic, Twist, queue_size=10)
        self.sub_ = rospy.Subscriber(sub__topic, Float64MultiArray, self.follow_callback)
        self.vel_msg = Twist()

        self.THRES_X = 0.2
        self.THRES_Y = 0.3
        self.TOLER_X = 0.2
        self.TOLER_Y = 0.3

        self.X_FAC = 0.5
        self.YAW_FAC = 1
        self.DBL_YAW_FAC = 1.5*self.YAW_FAC

        self.zero_up = 0.01
        self.zero_down = -0.01

    def follow_callback(self, msg, mode="left"):
        
        # if (msg.data == [0,0,0,0]):
        #     self.vel_msg.linear.x = 0
        #     self.vel_msg.angular.z = 0

        # else:
        curx = msg.data[0] 
        cury = msg.data[1] 
        newx = msg.data[2]  
        newy = msg.data[3] 

        diffx = 2*newx - curx
        diffy = 2*newy - cury
        self.vel_msg.linear.x = self.X_FAC*math.sqrt(diffx**2 + diffy**2)

        if diffx > 0:
            self.vel_msg.angular.z = self.YAW_FAC*math.atan(diffy/diffx)
        elif diffx < 0:
            self.vel_msg.angular.z = -self.DBL_YAW_FAC*math.atan(diffy/diffx)
        else:
            self.vel_msg.angular.z = abs(diffy)*self.YAW_FAC*(math.pi/2)

        # if (curx < X_UP and curx > X_LO and cury < Y_L and cury > Y_R) or (curx < X_UP and curx > X_LO and cury < -Y_L and cury > -Y_R):
        if mode == 'left':
            X_UP = self.THRES_X + self.TOLER_X
            X_LO = self.THRES_X - self.TOLER_X
            Y_R = self.THRES_Y + self.TOLER_X
            Y_L = self.THRES_Y - self.TOLER_X
            if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0

        # elif mode == 'right':
        #     X_UP = 0.5
        #     X_LO = -0.1
        #     Y_R = -0.7
        #     Y_L = 0.1
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'front':
        #     X_UP = 0.8
        #     X_LO = -0.1
        #     Y_R = -0.3
        #     Y_L = 0.3
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'front_left':
        #     X_UP = 0.8
        #     X_LO = 0
        #     Y_R = 0
        #     Y_L = 0.6
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'front_right':
        #     X_UP = 0.8
        #     X_LO = 0
        #     Y_R = -0.6
        #     Y_L = 0
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'back':
        #     X_UP = 0.1
        #     X_LO = -0.8
        #     Y_R = -0.3
        #     Y_L = 0.3
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'back_left':
        #     X_UP = 0.1
        #     X_LO = -0.8
        #     Y_R = 0
        #     Y_L = 0.6
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # elif mode == 'back_right':
        #     X_UP = 0.1
        #     X_LO = -0.8
        #     Y_R = -0.6
        #     Y_L = 0
        #     if self.person_in_range(curx, cury, X_UP, X_LO, Y_L, Y_R):
        #        self.vel_msg.linear.x = 0
        #        self.vel_msg.angular.z = 0

        # if diffx > zero_down and diffx < zero_up and diffy > zero_down and diffy < zero_up:
        #     self.vel_msg.linear.x = 0
        #     self.vel_msg.angular.z = 0
            
        print(self.vel_msg.linear.x, self.vel_msg.angular.z)
        self.pub_.publish(self.vel_msg)

    def person_in_range(self, curx, cury, X_UP, X_LO, Y_L, Y_R):
        if (curx < X_UP and curx > X_LO and cury < Y_L and cury > Y_R): # if user is within these boundaries, stop movement
            return True


if __name__ == '__main__':
    rospy.init_node('person_follow_py')
    Person_follow()
    rospy.spin()
