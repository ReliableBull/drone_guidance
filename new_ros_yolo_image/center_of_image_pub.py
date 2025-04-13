#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import math
from ultralytics import YOLO
from yolo_segmentation import YOLOSegmentation
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sklearn.linear_model import LinearRegression
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import LinkStates

# init point
point_msg = Point()
point_msg.x = -1
point_msg.y = -1
point_msg.z = -1

area_msg = Float32()
area_msg.data = 0.0


angle_truth_msg = Float32()
angle_truth_msg.data = -9999
angle_msg = Float32()
angle_msg.data = -9999

pose_msg = Point()

# 3개의 값을 담을 리스트를 초기화
window = []

target_points_x = []
target_points_y = []

# 평균 값을 저장할 리스트를 초기화
# averages = []
model = LinearRegression()
# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)

    if cv2.waitKey(1) & 0xFF == ord('s'):
        # 이미지 파일명 설정
        image_count += 1
        filename = os.path.join(save_dir, f'captured_image_{image_count}.png')
        # 이미지 저장
        cv2.imwrite(filename, frame)
        print(f'이미지 {filename}이 저장되었습니다.')

    # cv2.waitKey(1)

# calc ground truth
def calculate_angle2(point1, point2):
    global angle_truth_msg

    x1, y1, z1 = point1[0], point1[1], point1[2]
    x2, y2, z2 = point2[0], point2[1], point2[2]

    # 벡터 AB 구하기
    AB = [x2 - x1, y2 - y1, z2 - z1]
    
    if z1 > z2 :
        x3 = x1
        y3 = y1
        z3 = z2

        BC = [x3 - x2 , y3 - y2 , 0 ]
    else:
        x3 = x2
        y3 = y2
        z3 = z1
        BC = [x3 - x1 , y3 - y1 , 0 ]


    # (AB, 0, 0) 벡터와 (1, 0, 0) 벡터 사이의 각도 계산
    dot_product = AB[0] * BC[0] + AB[1] * BC[1] + AB[2] * BC[2]  # 내적은 AB 벡터의 x 성분과 (1, 0, 0) 벡터의 x 성분만 고려

    length_AB = math.sqrt(AB[0]**2 + AB[1]**2 + AB[2]**2)  # AB 벡터의 크기
    length_BC = math.sqrt(BC[0]**2 + BC[1]**2 + BC[2]**2)  # AB 벡터의 크기

    # 내적과 크기를 사용하여 각도 계산
    if length_AB > 0:
        cos_angle = dot_product / (length_AB * length_BC)
        # print(cos_angle)
        angle = math.degrees(math.acos(cos_angle))
        if angle > 90 and angle <= 180:
            angle = angle - 180
        # print(angle)
        angle_truth_msg.data = -angle
        return angle
    else:
        return None

# Callback 함수 정의
def link_states_callback(msg):
    if len(msg.pose) > 25:
        pose1 = msg.pose[8]  # 23번째 링크의 포즈 정보
        pose2 = msg.pose[10]  # 25번째 링크의 포즈 정보

        # 두 점 사이의 각도 계산
        point1 = [pose1.position.x, pose1.position.y, pose1.position.z]
        point2 = [pose2.position.x, pose2.position.y, pose2.position.z]

        angle = calculate_angle2(point1, point2)

        # if angle is not None:
            # rospy.loginfo("두 점 사이의 각도: {}도".format(angle))
        
        # print("--------------------")
        # print(point1[0] - point2[0],point1[1] - point2[1],point1[2] - point2[2])
        # print("--------------------")
        # print(point2)
def calculate_angle(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    # 두 점을 통해 만들어지는 벡터의 내적(dot product)을 계산합니다.
    dot_product = x1 * x2 + y1 * y2

    # 각 점의 벡터 길이를 계산합니다.
    magnitude1 = math.sqrt(x1 ** 2 + y1 ** 2)
    magnitude2 = math.sqrt(x2 ** 2 + y2 ** 2)

    # 두 벡터의 내적을 사용하여 코사인 값을 계산합니다.
    cos_theta = dot_product / (magnitude1 * magnitude2)

    # 아크코사인을 사용하여 라디안 단위의 각도를 계산합니다.
    angle_rad = math.acos(cos_theta)

    # 라디안을 도(degree)로 변환합니다.
    angle_deg = math.degrees(angle_rad)

    return angle_deg


# Define a callback for the Image message
def image_callback(img_msg):
 # log some info about the image topic
        #rospy.loginfo(img_msg.header)
        # Try to convert the ROS Image message to a CV2 Image
        global model
        global target_points_x
        global target_points_y

        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8") # color
            # cv_image = cv2.resize(cv_image , dsize=(0,0) ,fx=0.7 , fy=0.7)
            h,w,_ = cv_image.shape
            # w = 640
            # h = 480

            print(cv_image.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        bboxes, classes, segmentations, scores , flag= ys.detect(cv_image)
        target_points_x=[]
        target_points_y=[]
        if flag == 1:
            for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
                # print("bbox:", bbox, "class id:", class_id, "seg:", seg, "score:", score)
                
                # if score < 0.4:
                #   continue
                (x, y, x2, y2) = bbox
                # print(x,y,x2,y2)
                # if class_id == 4:
                #     cv2.rectangle(cv_image, (x, y), (x2, y2), (255, 0, 0), 2)

                #     cv2.polylines(cv_image, [seg], True, (0, 0, 255), 4)

                #     cv2.putText(cv_image, str(class_id), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                minLeftWing = 9999
                minRightWing = 9999
                leftWingX = -1
                leftWingY = -1
                rightWingX = -1
                rightWingY = -1

                # for t_x,t_y in seg:
                #   if t_y <= centerY:
                #     if t_x < centerX:
                #       if abs(t_x-x) < minLeftWing:
                #         minLeftWing = abs(x-t_x)
                #         leftWingX = t_x
                #         leftWingY = t_y

                #     if t_x > centerX:
                #       if abs(x2-t_x) < minRightWing:
                #         minRightWing = abs(t_x-x2)
                #         rightWingX = t_x
                #         rightWingY = t_y

                # experiments x,y,x2,y2
                x = x- 7
                y = y - 7
                x2 =x2 +7
                y2 = y2 +7

                centerX = (x + x2) / 2
                centerY = (y + y2) / 2


                l_centerX = (x2 -x ) / 2
                l_centerY = (y2 - y) / 2

                # cropped_img = cv_image[y: y2, x: x2]
                
                # gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
                # gray = cv2.GaussianBlur(gray, (5, 5), 0)
                # gray = cv2.bitwise_not(gray)
                # adaptive_threshold= cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 5)             

                width = abs(x2 - x) 
                height = abs(y2 - y)
                area = width * height

                _max = -9999
                _maxR = -9999
                t_x = 0
                t_y = 0
                # print(adaptive_threshold.shape)
                # if area < 6000:
                #   for j in range(y2 - y):                    
                #     for i in range(x2 - x):
                #       # if i>= adaptive_threshold.shape[0] or  j>= adaptive_threshold.shape[0] :
                #       #   break
                #       # _flag = 0
                #       # cv_image[y+j][x+i] =  adaptive_threshold[j][i]
                #       try:
                #         if adaptive_threshold[j][i] == 0:
                #           if j < int((y2-y)/2) and i < int((x2-x)/2):
                #             cv_image[y+j][x+i] =  adaptive_threshold[j][i]
                #             dis = math.sqrt((l_centerX - i)**2 + (l_centerY - j)**2)
                #             # print(dis)
                #             if dis > _max:
                #               _max = dis
                #               leftWingX = i
                #               leftWingY = j

                #           if j < int((y2-y)/2) and i > int((x2-x)/2):
                #             cv_image[y+j][x+i] =  adaptive_threshold[j][i]
                #             dis1 = math.sqrt((i - l_centerX )**2 + (l_centerY - j)**2)
                #             if dis1 > _maxR:
                #               _maxR = dis1
                #               rightWingX = i
                #               rightWingY = j
                #       except Exception as err:
                #         pass                     
                          
                          

                #         # _flag = 1
                #         # target_points_x.append(i)
                #         # target_points_y.append(j)

                        
                      

                #   # for j in range(y2 - y):                    
                #   #   for i in range(x2 - x):
                #   #   # _flag = 0
                    
                #   #     if adaptive_threshold[j][i] == 0:
                #   #       # _flag = 1
                #   #       target_points_x.append(i)
                #   #       target_points_y.append(j)

                #   #       cv_image[y+j][x+i] =  adaptive_threshold[j][i]
                #   #       break                                              
                #     # if _flag == 1:
                #     #   break
                #       # cv_image[y+j][x+i] =  adaptive_threshold[j][i]
                      
                #         # if (y+j) <= centerY:
                #         #   if (x+i) < centerX:
                #         #     if abs((x+i)-x) < minLeftWing:
                #         #       minLeftWing = abs(x-(i+x))
                #         #       leftWingX = i
                #         #       leftWingY = j

                #         #   if (x+i) > centerX:
                #         #     if abs(x2-(i+x)) < minRightWing:
                #         #       minRightWing = abs((x+i)-x2)
                #         #       rightWingX = i
                #         #       rightWingY = j

                #   # for j in range(y2 - y):
                #   #   for i in range(x2 - x):
                #   #     if adaptive_threshold[j][i] == 0:
                #   #       if (y+j) <= centerY:
                #   #         if (x+i) < centerX:
                #   #           if y+j < minLeftWing:
                #   #             minLeftWing = y+j
                #   #             leftWingX = i
                #   #             leftWingY = j

                #   #         if (x+i) > centerX:
                #   #           if y+j < minRightWing:
                #   #             minRightWing = y+j
                #   #             rightWingX = i
                #   #             rightWingY = j    

                #       # D

                cv2.rectangle(cv_image, (x, y), (x2, y2), (255, 0, 0), 1)

                # cv2.polylines(cv_image, [seg], True, (0, 0, 255), 1)

                # cv2.putText(cv_image, str(scores), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
               
                # print(centerX ,":::" ,centerY)
                cv2.circle(cv_image, (int(centerX), int(centerY)), 2, (255, 0, 0), -1)
                
                cv2.circle(cv_image, (int(w/2), int(h/2)), 2, (255, 0, 0), -1)

                # if area < 6000:
                #   cv2.circle(cv_image, (int(x+leftWingX), int(y+leftWingY)), 2, (0, 255, 0), -1)
                #   cv2.circle(cv_image, (int(x+rightWingX), int(y+rightWingY)), 2, (0, 255, 0), -1)


                point_msg.x = centerX
                point_msg.y = centerY
                point_msg.z = 1


                area_msg.data = area

                # target_points_x = np.array(target_points_x)
                # target_points_y = np.array(target_points_y)

                # x_reshaped = target_points_x.reshape(-1, 1)

                # model.fit(x_reshaped, target_points_y)

                # print(model.coef_[0])
                #left
                a_x = x+leftWingX
                a_y = y+leftWingY

                #right
                b_x = x+rightWingX
                b_y = y+rightWingY


                angle = calculate_angle((x+leftWingX,y+leftWingY) , (x2 ,y))
                

                # delta_x = b_x - a_x
                # delta_y = b_y - a_y

                # diff_deg = math.atan2(delta_y, delta_x)* 180 / 3.14

                # average = -9999

                # window.append(diff_deg)
                
                # if len(window) == 5:
                #   # 3개의 값이 있을 때 평균 계산
                #   average = sum(window) / len(window)
                #   # 첫 번째 값을 제거하여 3개의 값 유지
                #   window.pop(0)
                #   if abs(average) < 5:
                #     average = 0
                #   else:
                #     average = average *1 # gain

                #   cv2.putText(cv_image, str(round(average,1)), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

                # cv2.putText(cv_image, str(round(diff_deg,1)), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                # if area < 7000:
                #   angle_msg.data = average
                # else:
                #   angle_msg.data = 0.0
                # print(diff_deg)
                # cv2.putText(cv_image, str(diff_deg), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
                #print("left : " , leftWingY , " right : " , rightWingY )
                
                # print(_maxR)

        else:
          point_msg.z = -1



        show_image(cv_image)

def pose_callback(data):
  global pose_msg

  orientation = data.pose.orientation
  orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
  (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
  pose_msg.x = math.degrees(roll)
  pose_msg.y = math.degrees(pitch)
  pose_msg.z = math.degrees(yaw)


rospy.init_node('image_info')

  # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("run image info")

  # Initialize the CvBridge class
bridge = CvBridge()

save_dir = "./images"
image_count = len(os.listdir(save_dir))

ys = YOLOSegmentation("yolov8_drone.pt")

# subscriber
sub_image = rospy.Subscriber("/uav0/rrbot/camera1/image_raw", Image, image_callback)
 # "Gazebo/link_states" 토픽을 구독할 Subscriber 생성
rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)

# publisher
pub = rospy.Publisher('center_of_image', Point, queue_size=10)

pub_box_area = rospy.Publisher('area_of_box', Float32, queue_size=10)

pub_target_angle = rospy.Publisher('angle_of_target', Float32, queue_size=10)
pub_target_truth_angle = rospy.Publisher('angle_of_truth', Float32, queue_size=10)

pose_sub = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, pose_callback)
pose_pub = rospy.Publisher('target_drone_pose', Point, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0

rate = rospy.Rate(50)

while not rospy.is_shutdown():  
  pub.publish(point_msg)
  pub_box_area.publish(area_msg)
  pub_target_truth_angle.publish(angle_truth_msg)
  pub_target_angle.publish(angle_msg)
  pose_pub.publish(pose_msg)
  rate.sleep()
  # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
# while not rospy.is_shutdown():
#     rospy.spin()
