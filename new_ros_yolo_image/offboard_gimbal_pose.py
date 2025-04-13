#!/usr/bin/env python3

import rospy
import threading, requests, time
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped , Vector3, Point, TwistStamped
from mavros_msgs.msg import State , AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool , SetModeRequest , SetMavFrame , SetMavFrameRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix , Image
from cv_bridge import CvBridge, CvBridgeError
import datetime
from getkey import getkey



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
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_cb)
        rospy.Subscriber("/mavros/global_position/global" , NavSatFix , self.callBaclkGlobalPosition)
        

        # Initialize ROS Publisher
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.alt_pub = rospy.Publisher('/mavros/setpoint_position/rel_alt', Float32, queue_size=10)
        self.att_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

        self.current_target_vel_pub = rospy.Publisher("/current_vel", TwistStamped, queue_size=10)
        self.guidance_vel_pub = rospy.Publisher("/guidance_vel", Twist, queue_size=10)


        self.mission_start_pub = rospy.Publisher('/mission_flag', Float32, queue_size=10)

       
        # Initialize ROS Service
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Change frame
        self.set_mav_frame =rospy.ServiceProxy("/mavros/setpoint_velocity/mav_frame", SetMavFrame)     


        # Custom subscriber
        self.gimbal_angle_sub = rospy.Subscriber("/angle_of_gimbal", Point, self.callBackGimbalAngle)

        self.target_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.callBackTargetVel)

        self.area_of_box_sub = sub_point = rospy.Subscriber("/area_of_box", Float32, self.callBackAreaOfBox)

        self.area_of_box_sub = sub_point = rospy.Subscriber("/angle_of_target", Float32, self.callBackAngleOfTarget)

        # self.sub_point = rospy.Subscriber("/center_of_image", Point, self.move_gimbal)
        
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


        self.center_of_point = Point()

        self.target_real_vel = TwistStamped()
        # tracking 관련 target_lat = 35.8934302
        self.target_lat = None    
        self.target_lng = None        

        # Initialize PID Controllers
        self.vel_x_pid = PID(0.07, 0.01, 0.01, 3.0, -3.0, 1.0, -1.0, 0.1)
        self.vel_y_pid = PID(0.3, 0.01, 0.15, 1.5, -1.5, 1.0, -1.0, 0.1)
        self.vel_z_pid = PID(0.8, 0.01, 0.15, 3.0, -3.0, 1.0, -1.0, 0.1)
        # self.vel_y_pid = PID(0.8, 0.01, 0.3, 3.0, -3.0, 1.0, -1.0, 0.1)

        # z angular pid 
        # self.z_angular_pid = PID(0.2, 0.01, 0.03, 3.0, -3.0, 1.0, -1.0, 0.1) # only gimbal

        self.z_angular_pid = PID(0.01, 0.01, 0.01, 0.25, -0.25, 1.0, -1.0, 0.1) # with vision

        self.target_center_x = None
        self.target_center_ｙ = None

        self.image_center_x = None
        self.image_center_y = None

        # define some constants
        self.CONFIDENCE_THRESHOLD = 0.8
        self.GREEN = (0, 255, 0)  

        # gimbal angle
        self.gimbal_angle = Point()
        self.gimbal_angle.z = -1
        self.area_of_box = Float32()

        self.angle_of_target = Float32()

        # status
        self.status_cnt = 0

        self.start_flag = 0


        self.current_v_x = 0.0
        self.current_v_y = 0.0
        self.current_v_z = 0.0

        self.desired_v_x = 0.0
        self.desired_v_y = 0.0
        self.desired_v_z = 0.0

        self.mission_flag = 0.0
        # self.keyThread = threading.Thread(target=self.onKeyInput)
        # self.keyThread.daemon = True 
        # self.keyThread.start()



    # def onKeyInput(self, keyQueue):
    #     print('Ready for key input:')
    #     while (True):
    #         key = getkey()
    #         # keyQueue.put(key)
    #         print(key)


    # targetVelocity 
    def callBackTargetVel(self,data):
        self.target_real_vel = data


    # gimbal callback 
    def callBackGimbalAngle(self, data):
        # x : yaw
        # y : pitch
        self.gimbal_angle = data
        #print(self.gimbal_angle)

    # area of box callback    
    def callBackAreaOfBox(self, data):
        self.area_of_box = data
        # print(self.area_of_box)

        # area of box callback    
    def callBackAngleOfTarget(self, data):
        # print("!@#!@#@!#!",data)
        self.angle_of_target = data

    def callBaclkGlobalPosition(self,data):
        self.current_global_position = data
        #print(data)     
        #sub_topics_ready['global_pos'] = True

    #center of image callback
     # Define a callback for the Image message


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
        #print(odom_msg)
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
            # To do
            current_gimbal_yaw = self.gimbal_angle.x
            current_gimbal_pitch = self.gimbal_angle.y

            #  1 : ok
            # -1 : no
            status = -1
            status = self.gimbal_angle.z

            box_area = self.area_of_box.data

            target_angle = self.angle_of_target.data

            yaw_error = abs(current_gimbal_yaw + target_angle)

            y_error = -current_gimbal_pitch -3         

            # if abs(yaw_error) > 10:
            yaw_output = self.z_angular_pid.update(yaw_error*0.02)

            
            # else:
            #     yaw_output = 0.0

            if current_gimbal_yaw < 0: # rotate right
                yaw_output = -yaw_output

            print(target_angle ,":::",yaw_error,":::",yaw_output)

            # if abs(y_error) > 3:
            y_output = self.vel_z_pid.update(y_error*0.14)
            # else:
            #     y_output = 0.0

            # if target_angle == -9999:
            #     target_angle = 0

            # vel_y_output = self.vel_y_pid.update(target_angle)
            vel_y_output = 0.0
            if target_angle == -9999:
                target_angle = 0
            # else:
                # target_angle = 4.0
            # elif target_angle < -9:
            #     vel_y_output = 2
            # elif target_angle > 9:
            #     vel_y_output = -2
            # else:
            #     vel_y_output = 0.0
                

            # vel_x_output = abs(self.vel_x_pid.update(40-(abs(yaw_error))))
            # print(vel_x_output)
            # if abs(yaw_error ) < 5:
            #     vel_x_output = 0.0
            #print(90 - abs(yaw_error))
            # if abs(yaw_error) > 90:
            #     vel_x_output = 0.0

            # if box_area > 7000:
            #     vel_x_output = 0.0   
            # vel_x_output = 3.0
            # vel_y_output = 0.8
            
            speed = 2.5
            vel_x_output = 5.0
            if self.start_flag == 0:
                vel_x_output = 0.0
                vel_y_output = 0.0
                y_output = 0.0
                yaw_output = 0.0
                vel_y_output = 0.0
                target_angle = 0
                speed = 0

            # if status == -1:
            #     pass
            # else:
            #     self.start_flag = 1
            # print(self.status_cnt)
            if status == 1:
                self.status_cnt = self.status_cnt + 1

            if self.status_cnt == 1:
                self.start_flag = 1
                self.mission_flag = 1
                
            
            # vel_y_output = -vel_y_output
                # print("start")
                # self.status_cnt = 0

            # if self.status_cnt > 10 and self.start_flag == 0:
            #     self.start_flag = 1
            #     print("Ready")
                # vel_x_output = 0.0
                # yaw_output = 0.0
                # y_output = 0.0
                # vel_y_output = 0.0

            
            # origin
            # vel_msg = Twist()
            # vel_msg.linear.x = vel_x_output
            # vel_msg.linear.y = 0.0 # + : left - : right
            # vel_msg.linear.z = y_output
            # vel_msg.angular.x = 0.0
            # vel_msg.angular.y = 0.0
            # # + : LEFT
            # # - : RIGHT
            # vel_msg.angular.z = yaw_output
            # end origin

            # print(vel_y_output)

            # if abs(current_gimbal_yaw) < 20:
            #     current_gimbal_yaw = 0

            print(":::",target_angle)

            t_angle = 3
            # calc angle
            angle_deg = -current_gimbal_yaw - target_angle * t_angle

            # if closed target
            # if box_area >6000:
            #     angle_deg = 0

            angle_pitch_deg = current_gimbal_pitch-1 # -: down , + : up
            # m/s

            # 각도를 라디안으로 변환
            angle_yaw_rad = math.radians(angle_deg)
            angle_pitch_rad = math.radians(angle_pitch_deg)
            # angle_rad = angle_deg

            # x, y, z 방향으로의 속도 성분 계산
            # v_x = speed * math.cos(angle_yaw_rad)
            # v_y = speed * math.sin(angle_yaw_rad)
            v_x = speed * math.cos(angle_pitch_rad)*math.cos(angle_yaw_rad)
            v_y = speed * math.sin(angle_yaw_rad)*math.cos(angle_pitch_rad)
            v_z = speed * math.sin(angle_pitch_rad)

            if abs(v_x) < 1e-10:
                v_x = 0
            if abs(v_y) < 1e-10:
                v_y = 0

            if abs(v_z) < 1e-10:
                v_z = 0

            step = 0.05

            self.desired_v_x = v_x
            self.desired_v_y = v_y
            self.desired_v_z = v_z

            if self.current_v_x < self.desired_v_x:
                self.current_v_x = self.current_v_x +step
                if self.current_v_x > self.desired_v_x:
                    self.current_v_x = self.desired_v_x
            else:
                self.current_v_x = self.current_v_x -step
                if self.current_v_x < self.desired_v_x:
                    self.current_v_x = self.desired_v_x
            
            if self.current_v_y < self.desired_v_y:
                self.current_v_y = self.current_v_y +step
                if self.current_v_y > self.desired_v_y:
                    self.current_v_y = self.desired_v_y
            else:
                self.current_v_y = self.current_v_y -step
                if self.current_v_y < self.desired_v_y:
                    self.current_v_y = self.desired_v_y            
            
            if self.current_v_z < self.desired_v_z:
                self.current_v_z = self.current_v_z +step
                if self.current_v_z > self.desired_v_z:
                    self.current_v_z = self.desired_v_z
            else:
                self.current_v_z = self.current_v_z -step
                if self.current_v_z < self.desired_v_z:
                    self.current_v_z = self.desired_v_z            
            # self.current_v_y = self.current_v_x +step
            # self.current_v_z = self.current_v_x +step
            
            vel_msg = Twist()
            vel_msg.linear.x = self.current_v_x
            vel_msg.linear.y = self.current_v_y # + : left - : right
            vel_msg.linear.z = self.current_v_z
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            # + : LEFT
            # - : RIGHT
            vel_msg.angular.z = yaw_output
            # vel_msg.angular.z = 0.0

            
            # print(math.cos(math.radians(90)))
            # print(self.current_v_x , ":::" , self.current_v_y,":::",self.current_v_z)
            self.vel_pub.publish(vel_msg)

            self.current_target_vel_pub.publish(self.target_real_vel)

            self.guidance_vel_pub.publish(vel_msg)

            mFlag = Float32()
            mFlag.data = self.mission_flag 
            self.mission_start_pub.publish(mFlag) 
            rate.sleep()


if __name__ == "__main__":
    move_drone = MoveDrone()
    move_drone.change_mav_frame()
    move_drone.move_drone()
