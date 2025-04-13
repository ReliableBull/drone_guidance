#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# from utils import ARUCO_DICT, aruco_display
# gimbal_control 
import asyncio
from mavsdk import System
from mavsdk.gimbal import GimbalMode, ControlMode

from geometry_msgs.msg import Point


# yaw
errorY_last = 0
outputY =0

# pitch
errorP_last = 0
outputP =0

output_picth = 0
output_yaw = 0

current_angle = Point()

current_angle.x = -1
current_angle.y = -1
current_angle.z = -1
# Print "Hello!" to terminal
# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

  # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

  # Initialize the CvBridge class
bridge = CvBridge()

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
  # Define a function to show the image in an OpenCV Window



gimbal_pitch_pid = PID(0.3, 0.01, 0.03, 100.0, -100.0, 1.0, -1.0, 0.1) # with vision
gimbal_yaw_pid = PID(0.5, 0.01, 0.02, 100.0, -100.0, 1.0, -1.0, 0.1) # with vision
detect_flag= -1
gimbal_angle_pub = rospy.Publisher("/angle_of_gimbal", Point,queue_size=10)

def show_image(img):
    cv2.imshow("Image Window", img)
    
    cv2.waitKey(1)

  # Define a callback for the Image message
def image_callback(img_msg):

    # global errorY_last   
    # global outputY
    # global errorP_last   
    # global outputP

    # # log some info about the image topic
    # #rospy.loginfo(img_msg.header)

    #   # Try to convert the ROS Image message to a CV2 Image
    # try:
    #     cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8") # color
    # except CvBridgeError as e:
    #     rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
   # cv_image = cv2.transpose(cv_image)
   # cv_image = cv2.flip(cv_image,1)



    #convert to HSV
    #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    #lower_red = np.array([150, 50, 50])        # 빨강색 범위
    #upper_red = np.array([180, 255, 255])

    #mask = cv2.inRange(hsv, lower_red, upper_red)

    #print(np.transpose(mask.nonzero()))

    #points = np.transpose(mask.nonzero())
    
    #cx = int((points[:,0].min()+points[:,0].max())/2)
    #cy = int((points[:,1].min() + points[:,1].max())/2)

   # print("x: " , cx , " y : " , cy)


    #res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    #centerPoint = cv_image.shape
    #print(centerPoint)
    #print('img.shape ', cv_image.shape)

    # draw center point
    #cv_image = cv2.circle(cv_image , (cy,cx) , radius=0 , color=(255,0,0) , thickness=3)
    #cv_image = cv2.circle(cv_image , (int(centerPoint[1]/2),int(centerPoint[0]/2)) , radius=0 , color=(0,0,0) , thickness=3)

    #cv_image = cv2.rectangle(cv_image , (points[:,1].min() , points[:,0].min()),(points[:,1].max() , points[:,0].max()) ,color=(0,0,0) , thickness=2 )
      # Show the converted image

    h, w, _ = cv_image.shape
    print(cv_image)
    #width=1000
    #height = int(width*(h/w))
    #cv_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_CUBIC)
    corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)

    (m_x , m_y , detected_markers) = aruco_display(corners, ids, rejected, cv_image)

    #print(h , w)
    #print("-------")
    #print(m_y , m_x)

  
    # m_y , m_x -> marker center y , x
    # yaw PID
    # if aruco marker detected

    maxVal = 20
    if m_x != -1:

      # yaw
      error = int(w/2) - m_x
      d_error = (error - errorY_last)
      errorY_last = d_error  

      KP_y = 2.0
      KI_y = 0.0
      KD_y = 0.0000002

      if int(error) == 0:
        outputY = 0
      elif error > 0: # right marker
        outputY = KP_y * error + KD_y * d_error

        if outputY >= maxVal:
          outputY = maxVal
        outputY = -outputY
        
      else:
        outputY = KP_y * error + KD_y * d_error

        if outputY <= -maxVal:
          outputY = -maxVal
        outputY = -outputY


      # pitch
      errorP = int(h/2) - m_y
      p_error = (errorP - errorP_last)
      errorP_last = p_error  

      KP_p = 2.0
      KI_p = 0.0
      KD_p = 0.0000002

      if int(errorP) == 0:
        outputP = 0
      elif errorP > 0: # right marker
        outputP = KP_p * errorP + KD_p * p_error

        if outputP >= maxVal:
          outputP = maxVal
        outputP = outputP
        
      else:
        outputP = KP_p * errorP + KD_p * p_error

        if outputP <= -maxVal:
          outputP = -maxVal
        outputP = outputP


      # print(outputP)
          # pitch
      #errorP_last = 0
      #outputP =0


      #print(error)

    
    cv2.circle(detected_markers, (int(w/2), int(h/2)), 2, (255, 0, 0), -1)
    #drone.gimbal.set_pitch_rate_and_yaw_rate(0, -10)
    show_image( detected_markers)

# start gimbal control method

def center_callback(msg):

    global gimbal_pitch_pid
    global gimbal_yaw_pid

    global output_picth 
    global output_yaw 
    global detect_flag

    image_width = 640
    image_height = 480

    x = msg.x
    y = msg.y
    detect_flag = msg.z

    yaw_error =  image_width/2 - x 
    pitch_error =  y - image_height/2  

    
    output_picth = gimbal_pitch_pid.update(pitch_error)
    output_yaw = gimbal_yaw_pid.update(yaw_error)

    # print(str(output_picth) +":: " + str(output_yaw))

    if abs(yaw_error) < 3:
      output_yaw = 0.0

    if abs(pitch_error) < 3:
      output_picth = 0.0

    if detect_flag == -1:
        output_picth = output_yaw = 0


#    print(msg)

a = 0.0
async def run():
    # Init the drone

    global outputY
    global outputP
    global a

    global output_picth 
    global output_yaw 

    drone = System()
    await drone.connect(system_address="udp://:14599")

    # Start printing gimbal position updates
    print_gimbal_position_task = \
        asyncio.ensure_future(print_gimbal_position(drone))

    print("Taking control of gimbal")
    await drone.gimbal.take_control(ControlMode.PRIMARY)

    #print(outputY) 
    
    #await drone.gimbal.set_pitch_and_yaw(-90, 0)
    while 1:
        #a = a+1
        # print(output_picth,":::",output_yaw)
        await drone.gimbal.set_pitch_rate_and_yaw_rate(output_picth , output_yaw)
        await asyncio.sleep(0.000001)
        True

async def print_gimbal_position(drone):
    global gimbal_angle_pub
    global current_angle
    global detect_flag
    # Report gimbal position updates asynchronously
    # Note that we are getting gimbal position updates in
    # euler angles; we can also get them as quaternions
    async for angle in drone.telemetry.camera_attitude_euler():
      current_angle.x = angle.yaw_deg
      current_angle.y = angle.pitch_deg
      current_angle.z = detect_flag
      # print("OK")
      gimbal_angle_pub.publish(current_angle)
      # print("1")
      print(f"Gimbal pitch: {angle.pitch_deg}, yaw: {angle.yaw_deg}")

# end gimbal control method

  # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
#sub_image = rospy.Subscriber("/standard_vtol/c14540amera/rgb/image_raw", Image, image_callback)
# sub_image = rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)

#   # Initialize an OpenCV Window named "Image Window"
# cv2.namedWindow("Image Window", 1)

  # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
center_sub = rospy.Subscriber('center_of_image', Point, center_callback)
# print("GGG")
asyncio.run(run())
# print("GGG")
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    
    gimbal_angle_pub.publish(current_angle)
    rate.sleep()