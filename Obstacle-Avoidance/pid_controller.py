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
        rospy.Subscriber('/goal', Float64MultiArray, self.tracker_callback)
        rospy.spin()

        self.tracker_x = 0
        self.tracker_y = 0
        self.tracker_angle = 0

        self.tracker_distance = 0
        self.tracker_lost = False

        self.robot_x = 0
        self.robot_y = 0
        self.robot_angle = 0

        self.velocity_memory = 0
        self.lateral_memory = 0
        self.angular_memory = 0

        self.dt = 0.1

        self.drive_mode = drive
    
    def calc_distance_x_axis(self, tracker_x, tracker_y, robot_x, robot_y):

        tracker_distance = math.sqrt((robot_x - tracker_x)**2 + (robot_y - tracker_y)**2)
        if tracker_x < 0:
            tracker_distance = tracker_distance * -1
        
        return tracker_distance

    def calc_distance_y_axis(self, tracker_x, tracker_y, robot_x, robot_y):

        tracker_distance = math.sqrt((robot_x - tracker_x)**2 + (robot_y - tracker_y)**2)
        if tracker_y < 0:
            tracker_distance = tracker_distance * -1
        
        return tracker_distance
    
    def tracker_callback(self, msg):
        if len(msg.data) == 6:
            self.tracker_x = msg.data[0]
            self.tracker_y = msg.data[1]
            self.tracker_angle = msg.data[2]

            self.robot_x = msg.data[3]
            self.robot_y = msg.data[4]
            self.robot_angle = msg.data[5]
        else:
            print(f"Topic /goal receiving {len(msg.data)} values instead of 6.")

        if self.tracker_x == 0 and self.tracker_y == 0 and self.tracker_angle == 0:
            self.tracker_lost = True
            
        if self.drive_mode in ["diff"]:
            tracker_velocity = self.calc_distance_x_axis(tracker_x=self.tracker_x,
                                                         tracker_y=self.tracker_y,
                                                         robot_x=self.robot_x,
                                                         robot_y=self.robot_y)
            if tracker_velocity == 0: 
                distance_error = 0
            else:
                distance_error = tracker_velocity

            angle_error = self.tracker_angle - self.robot_angle

            self.pid_velocity = PIDController(0.2, 0.05, 0.085)
            control_signal_velocity = self.pid_velocity.calculate(distance_error,
                                                                  dt=self.dt)
            scale_factor = 1.5
            scaled_velocity = control_signal_velocity * scale_factor

            self.pid_angular = PIDController(0.25, 0, 0.05)
            control_signal_angular = self.pid_angular.calculate(angle_error,
                                                                dt=self.dt)
            scale_factor = 0.05
            scaled_angular = control_signal_angular * scale_factor

            print("Velocity:", scaled_velocity)
            print("Angular:", scaled_angular)

            if scaled_velocity != 0:
                self.velocity_memory = scaled_velocity
            if scaled_angular != 0:
                self.angular_memory = scaled_angular

        elif self.drive_mode in ["omni"]:
            tracker_velocity = self.calc_distance_x_axis(tracker_x=self.tracker_x,
                                                         tracker_y=self.tracker_y,
                                                         robot_x=self.robot_x,
                                                         robot_y=self.robot_y)
            
            tracker_lateral = self.calc_distance_y_axis(tracker_x=self.tracker_x,
                                                        tracker_y=self.tracker_y,
                                                        robot_x=self.robot_x,
                                                        robot_y=self.robot_y)
            
            if tracker_velocity == 0:
                distance_error = 0
            else:
                distance_error = tracker_velocity

            if tracker_lateral == 0:
                lateral_error = 0
            else:
                lateral_error = tracker_lateral

            angle_error = self.tracker_angle - self.robot_angle

            self.pid_velocity = PIDController(0.2, 0.05, 0.085)
            control_signal_velocity = self.pid_velocity.calculate(distance_error,
                                                                  dt=self.dt)
            scale_factor = 1.5
            scaled_velocity = control_signal_velocity * scale_factor

            self.pid_lateral = PIDController(0.15, 0.02, 0.085)
            control_signal_lateral = self.pid_lateral.calculate(lateral_error,
                                                                dt=self.dt)
            scale_factor = 0.5
            scaled_lateral = control_signal_lateral * scale_factor

            self.pid_angular = PIDController(0.25, 0, 0.05)
            control_signal_angular = self.pid_angular.calculate(angle_error,
                                                                dt=self.dt)
            scale_factor = 0.05
            scaled_angular = control_signal_angular * scale_factor

            print("Velocity:", scaled_velocity)
            print("Lateral:", scaled_lateral)
            print("Angular:", scaled_angular)

            if scaled_velocity != 0:
                self.velocity_memory = scaled_velocity
            if scaled_lateral != 0:
                self.lateral_memory = scaled_lateral
            if scaled_angular != 0:
                self.angular_memory = scaled_angular

        elif self.drive_mode in ["spin"]:
            tracker_velocity = self.calc_distance_x_axis(tracker_x=self.tracker_x,
                                                         tracker_y=self.tracker_y,
                                                         robot_x=self.robot_x,
                                                         robot_y=self.robot_y)
            if tracker_velocity == 0: 
                distance_error = 0
            else:
                distance_error = tracker_velocity

            angle_error = self.tracker_angle - self.robot_angle

            self.pid_velocity = PIDController(0.2, 0.05, 0.085)
            control_signal_velocity = self.pid_velocity.calculate(distance_error,
                                                                  dt=self.dt)
            scale_factor = 1.5
            scaled_velocity = control_signal_velocity * scale_factor

            self.pid_angular = PIDController(0.3, 0, 0.05)
            control_signal_angular = self.pid_angular.calculate(angle_error,
                                                                dt=self.dt)
            scale_factor = 0.06
            scaled_angular = control_signal_angular * scale_factor

            print("Velocity:", scaled_velocity)
            print("Angular:", scaled_angular)

            if scaled_velocity != 0:
                self.velocity_memory = scaled_velocity
            if scaled_angular != 0:
                self.angular_memory = scaled_angular

        else:
            print("Drive mode is not differential, omni or spin & go. Please specify drive mode again.")

        movement_commands_msg = Float64MultiArray()

        if self.drive_mode in ["diff"]:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, scaled_angular]
            else:
                movement_commands_msg.data = [scaled_velocity, scaled_angular]
            self.movement_commands_pub.publish(movement_commands_msg)
        
        elif self.drive_mode in ["omni"]:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, scaled_angular, scaled_lateral]
            else:
                movement_commands_msg.data = [scaled_velocity, scaled_angular, scaled_lateral]
            self.movement_commands_pub.publish(movement_commands_msg)

        elif self.drive_mode in ["spin"]:
            if self.tracker_lost is True:
                movement_commands_msg.data = [self.velocity_memory, scaled_angular]
            else:
                if angle_error in range(-10,10):
                    movement_commands_msg.data = [scaled_velocity, scaled_angular]
                else:
                    movement_commands_msg.data = [0, scaled_angular]
                self.movement_commands_pub.publish(movement_commands_msg)

if __name__ == '__main__':
    rospy.init_node('pid_movement')
    drive = rospy.get_param("/drive")
    PIDMovement()
    rospy.spin()
