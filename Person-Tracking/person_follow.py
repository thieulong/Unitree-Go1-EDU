#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# assume person is standing on the left and in front of the robot
THRES_X = 0.2
THRES_Y = 0.3
TOLER_X = 0.3
TOLER_Y = 0.4
X_UP = THRES_X + TOLER_X
X_LO = THRES_X - TOLER_X
Y_R = THRES_Y - TOLER_Y
Y_L = THRES_Y + TOLER_Y

X_FAC = 0.9
YAW_FAC = 0.9

class Person_follow:
    def __init__(self):
        pub__topic = "/cmd_vel"
        sub__topic = "/person_lock_on"

        self.pub_ = rospy.Publisher(pub__topic, Twist, queue_size=10)
        self.sub_ = rospy.Subscriber(sub__topic, Float64MultiArray, self.follow_callback)
        self.vel_msg = Twist()

    def follow_callback(self, msg):

        curx = msg.data[0]
        cury = msg.data[1]

        

        if (cury >= Y_L):
            if (curx < X_LO):
                self.vel_msg.linear.x = -(curx - X_LO)/X_FAC
                self.vel_msg.angular.z = 1.0/YAW_FAC
                # self.pub_.publish(self.vel_msg)
                # while (cury > Y_L):    
                #     continue
            elif (curx > X_UP):
                self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
                self.vel_msg.angular.z = (cury - Y_L)/ YAW_FAC * 0.9
            else:
                self.vel_msg.linear.x = 0.5
                self.vel_msg.angular.z = 0.8    
        elif (cury <= Y_R):
            if (curx < X_LO):
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
            elif (curx > X_UP):
                self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
                self.vel_msg.angular.z = (cury - Y_R)/ YAW_FAC * 1.5
            else:
                self.vel_msg.angular.z = -1.0
        else:
            self.vel_msg.angular.z = 0
            if (curx < X_LO):
                self.vel_msg.linear.x = (curx - X_LO)/ X_FAC
            elif (curx > X_UP):
                self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
            else:
                self.vel_msg.linear.x = 0

        

        if (curx < X_UP and curx > X_LO and cury < Y_L and cury > Y_R) or (curx < X_UP and curx > X_LO and cury < -Y_L and cury > -Y_R):
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        self.pub_.publish(self.vel_msg)


        



if __name__ == '__main__':
    rospy.init_node('person_follow_py')
    Person_follow()
    rospy.spin()