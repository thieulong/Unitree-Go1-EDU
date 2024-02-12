#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math

# operation_mode
# 1 - Differential drive
# 2 - Omni drive
# 3 - Spin and go
operation_mode = 1

# positions of target to be tracked
# 1 - behind
# 2 - side 
pos = 1

set_lateral = 0.1 # m
set_angle = 5 # degrees


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

        self.current_tracker_x = 0
        self.current_tracker_y = 0

        self.tracker_angle = 0
        self.tracker_distance = 0
        self.tracker_lateral = 0

        self.tracker_lost = False
        self.velocity_memory = 0
        self.angular_memory = 0
        self.lateral_memory = 0

    def calc_angle(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        angle = abs(math.degrees(math.atan(x/(y+0.01))))
        angle = round(angle)
        tracker_angle = 0
        if x > 0 and y > 0:
            tracker_angle = 90 - angle
        elif x < 0 and y > 0:
            tracker_angle = 90 + angle
        elif x < 0 and y < 0:
            tracker_angle = 180 + (90 - angle)
        elif x > 0 and y < 0:
            tracker_angle = 270 + angle  
        return tracker_angle
    
    def calc_distance_x_axis(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        # tracker_distance = math.sqrt((x)**2 + (y)**2)
        tracker_distance = x
        # if x < 0:
        #     tracker_distance = tracker_distance * -1
        
        return tracker_distance

    def calc_distance_y_axis(self, tracker_coords):
        x = tracker_coords[0]
        y = tracker_coords[1]
        # tracker_distance = math.sqrt((x)**2 + (y)**2)
        tracker_distance = y
        # if y < 0:
        #     tracker_distance = tracker_distance * -1
        
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

        if self.current_tracker_x == 0 and self.current_tracker_y == 0:
            self.tracker_lost = True
        else:
            tracker_coords = [self.current_tracker_x, self.current_tracker_y]
        dt = 0.1

        if operation_mode == 1 or operation_mode == 3:
            self.tracker_angle = self.calc_angle(tracker_coords=tracker_coords)
            self.tracker_distance = self.calc_distance_x_axis(tracker_coords=tracker_coords)
            if self.tracker_distance == 0: distance_error = 0
            else: distance_error = self.tracker_distance - set_distance

        elif operation_mode == 2:
            self.tracker_angle = self.calc_angle(tracker_coords=tracker_coords)
            self.tracker_distance = self.calc_distance_x_axis(tracker_coords=tracker_coords)
            self.tracker_lateral = self.calc_distance_y_axis(tracker_coords=tracker_coords)
            if self.tracker_distance == 0: distance_error = 0
            else: distance_error = self.tracker_distance - set_distance # frame_id: laser is set far away from base_link?
            lateral_error = self.tracker_lateral - set_lateral

            self.pid_lateral = PIDController(0.15, 0.02, 0.085)
            control_signal_lateral = self.pid_lateral.calculate(lateral_error, dt=dt)
            scale_factor = 0.5
            scaled_lateral = control_signal_lateral * scale_factor

            if scaled_lateral != 0:
                self.lateral_memory = scaled_lateral

        if self.tracker_angle == 0:
            angle_error = 0
        else:
            angle_error = self.tracker_angle - set_angle 
            if angle_error > 180:
                angle_error = -(360 - self.tracker_angle) + set_angle

        if operation_mode == 1 or operation_mode == 2:

            self.pid_velocity = PIDController(0.2, 0.05, 0.085)
            control_signal_velocity = self.pid_velocity.calculate(distance_error, dt=dt)
            scale_factor = 1
            scaled_velocity = control_signal_velocity * scale_factor

            self.pid_angular = PIDController(0.25, 0, 0.05)
            control_signal_angular = self.pid_angular.calculate(angle_error, dt=dt)
            scale_factor = 0.05
            scaled_angular = control_signal_angular * scale_factor

        if operation_mode == 3:
            self.pid_velocity = PIDController(0.2, 0.05, 0.085)
            control_signal_velocity = self.pid_velocity.calculate(distance_error, dt=dt)
            scale_factor = 1.2
            scaled_velocity = control_signal_velocity * scale_factor

            self.pid_angular = PIDController(0.3, 0, 0.05)
            control_signal_angular = self.pid_angular.calculate(angle_error, dt=dt)
            scale_factor = 0.06
            scaled_angular = control_signal_angular * scale_factor

        if scaled_velocity != 0:
            self.velocity_memory = scaled_velocity
        if scaled_angular != 0:
            self.angular_memory = scaled_angular

        print("Velocity:", scaled_velocity)
        print("Angular:", scaled_angular)

        movement_commands_msg = Float64MultiArray()

        if operation_mode == 1:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, self.angular_memory]
            else:
                movement_commands_msg.data = [scaled_velocity, scaled_angular]
            self.movement_commands_pub.publish(movement_commands_msg)
        
        elif operation_mode == 2:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, self.angular_memory, self.lateral_memory]
            else:
                movement_commands_msg.data = [scaled_velocity, scaled_angular, scaled_lateral]
            self.movement_commands_pub.publish(movement_commands_msg)

        elif operation_mode == 3:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, self.angular_memory]
            else:
                if angle_error in range(-10,10):
                    movement_commands_msg.data = [scaled_velocity, scaled_angular]
                else:
                    movement_commands_msg.data = [0, scaled_angular]
                self.movement_commands_pub.publish(movement_commands_msg)

if __name__ == '__main__':
    rospy.init_node('pid_movement')
    PIDMovement()
    rospy.spin()
