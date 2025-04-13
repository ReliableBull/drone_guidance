#!/usr/bin/env python3

import rospy
import threading, requests, time
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped , Vector3, Point
from mavros_msgs.msg import State , AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool , SetModeRequest , SetMavFrame , SetMavFrameRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix , Image
from cv_bridge import CvBridge, CvBridgeError
import datetime
import threading



class PID:
    def __init__(self, kp, ki, kd, max_output, min_output, max_integ, min_integ, sample_time):
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        self.max_output = max_output  # Maximum Output
        self.min_output = min_output  # Minimum Output
        self.max_integ = max_integ  # Maximum Integral Term
        self.min_integ = min_integ  # Minimum Integral Term
        self.sample_time = sample_time  # Sample Time

        self.target = 0.0  # Target Value
        self.integ = 0.0  # Integral Term
        self.last_error = 0.0  # Last Error
        self.last_time = rospy.Time.now()  # Last Time

    def update(self, feedback_value):
        error = self.target - feedback_value  # Error
        dt = (rospy.Time.now() - self.last_time).to_sec()  # Time Step

        if dt == 0:
            dt = 0.003

        # Proportional Term
        P = self.kp * error

        # Integral Term
        self.integ += error * dt
        self.integ = max(self.min_integ, min(self.max_integ, self.integ))
        I = self.ki * self.integ

        # Derivative Term
        D = self.kd * (error - self.last_error) / dt

       
        # PID Output
        output = P + I + D

        # print(self.min_output,"::",self.max_output,":::",output)
        output = max(self.min_output, min(self.max_output, output))

        # Update Last Error and Last Time
        self.last_error = error
        self.last_time = rospy.Time.now()

        return output

class MoveDrone:
    def __init__(self):
        rospy.init_node('move_drone', anonymous=True)

        
           # CvBridge
        self.bridge = CvBridge()  

        # Initialize ROS Subscriber
        rospy.Subscriber('/uav1/mavros/state', State, self.state_cb)
        rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, self.position_cb)
        rospy.Subscriber("/uav1/mavros/global_position/global" , NavSatFix , self.callBaclkGlobalPosition)
        

        # Initialize ROS Publisher
        self.vel_pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.alt_pub = rospy.Publisher('/uav1/mavros/setpoint_position/rel_alt', Float32, queue_size=10)
        self.att_pub = rospy.Publisher("/uav1/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

       
        # Initialize ROS Service
        self.arm_service = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        self.flight_mode_service = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)

        # Change frame
        self.set_mav_frame =rospy.ServiceProxy("/uav1/mavros/setpoint_velocity/mav_frame", SetMavFrame)     


        # # Custom subscriber
        # self.gimbal_angle_sub = rospy.Subscriber("/angle_of_gimbal", Point, self.callBackGimbalAngle)

        # self.area_of_box_sub = sub_point = rospy.Subscriber("/area_of_box", Float32, self.callBackAreaOfBox)

        # Initialize Variables
        self.current_state = State()
        self.current_pose = None
        self.current_global_position = None
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "home"
        self.target_pose.pose.position.x = 10.0
        self.target_pose.pose.position.y = 10.0
        self.target_pose.pose.position.z = 0.0
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        
        self.frame_set_mode = SetMavFrameRequest()
        self.frame_set_mode.mav_frame = 8

        self.att_msg = AttitudeTarget()
        self.att_msg.header.stamp = rospy.Time.now()
        self.att_msg.type_mask = 128  # 롤, 피치, 요륙 속도 제어


        # tracking 관련 target_lat = 35.8934302
        self.target_lat = None    
        self.target_lng = None        

        # Initialize PID Controllers
        self.vel_x_pid = PID(0.2, 0.01, 0.01, 3.0, 0.0, 1.0, -1.0, 0.1)
        self.y_pid = PID(0.2, 0.01, 0.01, 1.0, -1.0, 1.0, -1.0, 0.1)

        # z angular pid 
        self.z_angular_pid = PID(0.2, 0.01, 0.01, 0.3, -0.3, 1.0, -1.0, 0.1)

        self.target_center_x = None
        self.target_center_ｙ = None

        self.image_center_x = None
        self.image_center_y = None

        # define some constants
        self.CONFIDENCE_THRESHOLD = 0.8
        self.GREEN = (0, 255, 0)  

        # gimbal angle
        self.gimbal_angle = Point()
        self.area_of_box = Float32()

        self.algular_z = 0.0

        self.input_thread = threading.Thread(target=self.user_input_thread)
        self.input_thread.daemon = True
        self.input_thread.start()

    def callBaclkGlobalPosition(self,data):
        self.current_global_position = data
        #print(data)     
        #sub_topics_ready['global_pos'] = True

    def setOffboard(self):
        last_req = rospy.Time.now()
        
        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if((self.flight_mode_service.call(self.offb_set_mode).mode_sent == True)):
                    #self.flight_mode_service.call(self.offb_set_mode)
                    rospy.loginfo("OFFBOARD enabled")
                    break
            # else:
            #     break

    def state_cb(self, state_msg):
        self.current_state = state_msg

    def position_cb(self, odom_msg):
        self.current_pose = odom_msg.pose.pose    

    def takeoff(self):
        rospy.loginfo("Taking off")
        # while not rospy.is_shutdown():
        #     # print(self.current_state.mode)
        #     # if self.current_pose.position.z >= 3.0:
        #     #     break
        #     # self.alt_pub.publish(3.0)
        #     vel_msg = Twist()
        #     vel_msg.linear.x = 1500.0
        #     vel_msg.linear.y = 0.0
        #     vel_msg.linear.z = 2100.0
        #     vel_msg.angular.x = 0.0
        #     vel_msg.angular.y = 0.0
        #     vel_msg.angular.z = 0.0
        #     self.vel_pub.publish(vel_msg)
        #     rospy.sleep(0.1)>

    def change_mav_frame(self):
        frame_set_mode = SetMavFrameRequest()
        frame_set_mode.mav_frame = 8              
        
        while not rospy.is_shutdown():
            if (self.set_mav_frame.call(frame_set_mode).success == True):
               rospy.loginfo("update mav frame")
               break

        return True

    def move_drone(self):
        rospy.loginfo("Moving Drone")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # # To do
            # current_gimbal_yaw = self.gimbal_angle.x
            # current_gimbal_pitch = self.gimbal_angle.x

            # box_area = self.area_of_box.data

            # yaw_error = abs(current_gimbal_yaw)

            
            # if abs(yaw_error) > 6:
            #     yaw_output = self.z_angular_pid.update(yaw_error*0.1)
            # else:
            #     yaw_output = 0.0

            # if current_gimbal_yaw < 0: # rotate right
            #     yaw_output = -yaw_output

            # #print(yaw_output)
            # vel_x_output = self.vel_x_pid.update(-abs(90 - abs(yaw_error)))

            # #print(90 - abs(yaw_error))
            # # if abs(yaw_error) > 90:
            # #     vel_x_output = 0.0

            # # if box_area > 7000:
            # #     vel_x_output = 0.0

            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 2.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            # + : LEFT
            # - : RIGHT
            # vel_msg.angular.z = self.algular_z

            #print(vel_x_output)
            self.vel_pub.publish(vel_msg)
            rate.sleep()

    def user_input_thread(self):
        while True:
            user_input = input()

            self.algular_z = float(user_input)
            print(user_input)


# print("!23")
if __name__ == "__main__":
    print("start")
    move_drone = MoveDrone()
    move_drone.change_mav_frame()
    move_drone.move_drone()
# print("WER")
