#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class PIDMovement:
    def __init__(self):
        self.movement_commands_pub = rospy.Publisher('/movement_commands', Float64MultiArray, queue_size=1)
        rospy.Subscriber('/person_tracking', Float64MultiArray, self.tracker_callback)
        rospy.spin()

        self.pid_v_kp = 0.2
        self.pid_v_ki = 0.05
        self.pid_v_kd = 0.085

        self.pid_omega_kp = 0.1
        self.pid_omega_ki = 0.01
        self.pid_omega_kd = 0.05

        self.current_tracker_x = 0
        self.current_tracker_y = 0

        self.tracker_angle = 0
        self.tracker_distance = 0

    def calc_angle(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        angle = abs(math.degrees(math.atan(x/(y+0.01))))
        angle = round(angle)
        if x < 0 and y < 0:
            tracker_angle = angle
        elif x < 0 and y > 0:
            tracker_angle = 90 + (90 - angle)
        elif x > 0 and y > 0:
            tracker_angle = 180 + angle
        elif x > 0 and y < 0:
            tracker_angle = 270 + (90 - angle)
        
        return tracker_angle
    
    def calc_distance(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        tracker_distance = math.sqrt((x)**2 + (y)**2)
        if x < 0:
            tracker_distance = tracker_distance * -1
        
        return tracker_distance
    
    def tracker_callback(self, msg):
        if len(msg.data) == 2:
            self.current_tracker_x = msg.data[0]
            self.current_tracker_y = msg.data[1]
        elif len(msg.data) == 4:
            self.current_tracker_x = msg.data[2]
            self.current_tracker_y = msg.data[3]
        else:
            pass
        
        tracker_coords = [self.current_tracker_x, self.current_tracker_y]
        self.tracker_angle = self.calc_angle(tracker_coords=tracker_coords)
        self.tracker_distance = self.calc_distance(tracker_coords=tracker_coords)

        distance_error = 0 - self.tracker_distance
        angle_error = 180 - self.tracker_angle
        dt = 0.1

        self.pid_velocity = PIDController(kp=self.pid_v_kp, ki=self.pid_v_ki, kd=self.pid_v_kd)
        control_signal_velocity = self.pid_velocity.calculate(distance_error, dt=dt)

        self.pid_angular = PIDController(kp=self.pid_omega_kp, ki=self.pid_omega_ki, kd=self.pid_omega_kd)
        control_signal_angular = self.pid_angular.calculate(angle_error, dt=dt)

        print("Velocity:", control_signal_velocity)
        print("Angular:", control_signal_angular)

        movement_commands_msg = Float64MultiArray()
        movement_commands_msg.data = [control_signal_velocity, control_signal_angular]
        self.movement_commands_pub.publish(movement_commands_msg)


if __name__ == '__main__':
    rospy.init_node('pid_movement')
    PIDMovement()
    rospy.spin()
