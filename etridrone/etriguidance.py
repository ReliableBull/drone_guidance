#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool , SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import tf.transformations
import threading, requests, time
import math
from geopy.distance import geodesic

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

        # Proportional Term
        P = self.kp * error

        # Integral Term
        self.integ += error * dt
        self.integ = max(self.min_integ, min(self.max_integ, self.integ))
        I = self.ki * self.integ

        # Derivative Term
        #D = self.kd * (error - self.last_error) / dt
        D = 0

        # PID Output
        output = P + I + D
        output = max(self.min_output, min(self.max_output, output))

        # Update Last Error and Last Time
        self.last_error = error
        self.last_time = rospy.Time.now()

        return output

class MoveDrone:
    def __init__(self):
        rospy.init_node('move_drone', anonymous=True)

        # Initialize ROS Subscriber
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_cb)
        rospy.Subscriber("/mavros/global_position/global" , NavSatFix , self.callBaclkGlobalPosition)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Initialize ROS Publisher
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.alt_pub = rospy.Publisher('/mavros/setpoint_position/rel_alt', Float32, queue_size=10)
       
        # Initialize ROS Service
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)


        # Initialize ROVER Position
        target_state_gps = rospy.Subscriber("rover/mavros/global_position/global" , NavSatFix , callback = self.targetGlobalPosition)
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

        # tracking 관련 target_lat = 35.8934302
        self.target_lat = None    
        self.target_lng = None  

        # target 이전 좌표
        self.prev_target_lat = None    
        self.prev_target_lng = None
        self.destination=None
       
        self.yaw = None

        # Initialize PID Controllers
        self.x_pid = PID(1.5, 0.00, 0.01, 6.5, -6.5, 1.0, -1.0, 0.1)
        self.y_pid = PID(1.5, 0.00, 0.01, 6.5, -6.5, 1.0, -1.0, 0.1)
        self.z_pid = PID(0.2, 0.00, 0.01, 0.5, -0.5, 1.0, -1.0, 0.1)


        # rover
        self.target_global_position = NavSatFix()  # rover global position

        self.begin = time.time()
        self.end = 0


#        self.t1 = threading.Thread(target=self.getHtml)
#        self.t1.start()

    # ROVER gps position Callback
    def targetGlobalPosition(self,data):
        # global target_global_position
        # #print(type(data))
        # target_global_position = data
       
        self.end = time.time()

        result = self.end - self.begin

        result = round(result,3)

        if result >= 0.5:
            self.target_global_position = data
            self.begin = self.end
           
        #self.target_global_position = data


        # self.target_lat = float(_json['results'][0]['g_lat'])
        # self.target_lng = float(_json['results'][0]['g_lng'])

        self.target_lat = self.target_global_position.latitude # 위도
        self.target_lng = self.target_global_position.longitude # 경도


        # global sp_glob
        # #sp_glob.altitude = target_global_position.altitude
        # sp_glob.altitude = 5
        # sp_glob.latitude = target_global_position.latitude
        # sp_glob.longitude = target_global_position.longitude
        # #print(calcAngle())
        # #sp_glob.yaw = calcAngle()
        # #sp_glob.yaw = Azimuth()
        # #sp_glob.yaw = 0

        # sp_glob.yaw = 90 * 3.14 / 180
        # sp_glob.type_mask = 3064
        # sp_glob.coordinate_frame = 6


    def pose_callback(self,msg):
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_radian = euler[2]  

        yaw_degree = math.degrees(yaw_radian)

        yaw_degree = (-yaw_degree + 360) % 360

        self.yaw = (yaw_degree + 90 ) % 360

    #rospy.loginfo(yaw_degree)
    # 여기서 yaw 각도를 추출합니다.

    def callBaclkGlobalPosition(self,data):
        #print(data)    
        self.current_global_position = data

        #sub_topics_ready['global_pos'] = True

    # heading 각도 계산
    # 시계방향 : 0
    # 반시계방향 : 1
    def rotate_to_desired_heading(self, current_heading, desired_heading):
        clockwise_rotation = desired_heading - current_heading
        counter_clockwise_rotation = current_heading - desired_heading

        if clockwise_rotation < 0:
            clockwise_rotation += 360

        if counter_clockwise_rotation < 0:
            counter_clockwise_rotation += 360

        if clockwise_rotation <= counter_clockwise_rotation:
            rotation_direction = 0  #"clockwise"
            rotation_angle = clockwise_rotation
        else:
            rotation_direction = 1 #"counter-clockwise"
            rotation_angle = counter_clockwise_rotation

        # 시계 , 반시계 방향 및 rotation_angle
        return rotation_direction, rotation_angle


    # 현재 position <-> Target position 각도 계산
    def calcAngle(self,current_lat , current_lng , target_lat , target_lng):
        lat1 = current_lat
        lon1 = current_lng

        # 목적지
        lat2 = target_lat
        lon2 = target_lng

        # 위도, 경도를 라디안 단위로 변환
        pi_1 = lat1 * math.pi / 180
        pi_2 = lat2 * math.pi / 180
        gamma_1 = lon1 * math.pi / 180
        gamma_2 = lon2 * math.pi / 180

        y = math.sin(gamma_2 - gamma_1) * math.cos(pi_2)
        x = math.cos(pi_1) * math.sin(pi_2) - math.sin(pi_1) * math.cos(pi_2) * math.cos(gamma_2 - gamma_1)
        theta = math.atan2(y, x); # 방위각 (라디안)

        bearing = (theta * 180 / math.pi + 360) % 360; # 방위각 (디그리, 정규화 완료)
       
        # return (bearing * math.pi / 180)
        return bearing

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

    def arm(self):
        #self.setOffboard()
       
        rospy.loginfo("Arming Drone")
       
        while not rospy.is_shutdown():
            if self.current_state.armed:
                break
            self.arm_service(True)
            rospy.sleep(1)
     

    def takeoff(self):
        rospy.loginfo("Taking off")
        while not rospy.is_shutdown():
            # print(self.current_state.mode)
            # if self.current_pose.position.z >= 3.0:
            #     break
            # self.alt_pub.publish(3.0)

            vel_msg = Twist()
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            rospy.sleep(0.1)

    def move_drone(self):
        rospy.loginfo("Moving Drone")

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            # f = open('./current_lat.txt', 'a')
            # f1 = open('./current_lng.txt', 'a')

            # f2 = open('./target_lat.txt', 'a')
            # f3 = open('./target_lng.txt', 'a')
            if self.current_pose is None:
                rospy.loginfo("Waiting for pose update...")
                continue

            if self.current_global_position is None:
                rospy.loginfo("Waiting for global position update...")
                continue

            # if self.target_lat is None:
            #     rospy.loginfo("Waiting for target position update...")
            #     continue
            # Update Target Position
            # x_error = self.target_pose.pose.position.x - self.current_pose.position.x
            # y_error = self.target_pose.pose.position.y - self.current_pose.position.y
            # z_error = self.target_pose.pose.position.z - self.current_pose.position.z
           
            #target_lat = 35.8934302
            #target_lng = 128.6134666

            # x_error = target_lat -self.current_global_position.latitude
            # y_error = target_lng -self.current_global_position.longitude

            # pointA : 이전 타겟 좌표
            # pointB : 현재 타겟 좌표
            # if(self.prev_target_lat == None):
            #     pointA = (self.target_lat , self.target_lng)
            #     pointB = (self.target_lat , self.target_lng)
            # else:              
            #     pointA = (self.prev_target_lat , self.prev_target_lng)
            #     pointB = (self.target_lat , self.target_lng)

            # comment
        #     pointA = (self.current_global_position.latitude ,self.current_global_position.longitude)

        #     pointB = (self.target_lat ,self.target_lng)

        #     # 이동 방향(베어링) 계산
        #     bearing = self.calculate_bearing(pointB, pointA)
           
        #     # print(pointA)
        #     # print(pointB)
        #     # print("bearing : " , bearing)
           
        #     # 이동 방향 반대쪽 5m 지점 좌표 계산
        #     destination_point = self.calculate_destination_coordinate(pointB, bearing, 5)

        #     # 타겟과 현재 드론의 거리 추출
        #     diff_distance = geodesic((self.prev_target_lat , self.prev_target_lng), (self.target_lat , self.target_lng)).meters
        #      #if(diff_distance > 1.5):                        
           
        #     #if diff_distance != 0.0:
        #         #print(diff_distance)

        #     # 타겟의 이전 좌표와 현재 좌표의 움직임의 차이가 1.0m 보다 크다면 목표점 갱신
        #     #if self.prev_target_lat != self.target_lat and  diff_distance > 1.5 :
        #     #if diff_distance > 1.5 :
        #     if self.prev_target_lat != self.target_lat:
        #         self.destination = destination_point
               
        #     if self.destination == None:
        #          continue

        #     x_error , y_error = self.distance_on_xy(self.destination[0] , self.destination[1]  ,self.current_global_position.latitude, self.current_global_position.longitude)

        #     self.prev_target_lat = self.target_lat
        #     self.prev_target_lng = self.target_lng

        #     # 헤딩각을 위한 계산
        #     diff_angle = self.calcAngle(self.current_global_position.latitude, self.current_global_position.longitude,self.target_lat , self.target_lng )
           
        #     # 타겟과 현재 드론의 거리 추출
        #     diff_distance = geodesic((self.current_global_position.latitude , self.current_global_position.longitude) , (self.target_lat,self.target_lng)).meters
           
        #     print("현재 거리 : " , diff_distance )
        #     # f.write(str(self.current_global_position.latitude)+'\n')
        #     # f1.write(str(self.current_global_position.longitude)+'\n')

        #     # f2.write(str(self.target_lat)+'\n')
        #     # f3.write(str(self.target_lng)+'\n')

        #     #    # 파일 닫기
        #     # f2.close()
        #     # f3.close()
        #     #print(x_error , y_error)
           
        #     x_output = self.x_pid.update(x_error)
        #     y_output = self.y_pid.update(y_error)



           
        #     if self.yaw != None:
        #         (heading_rotation , heading_error) = self.rotate_to_desired_heading(self.yaw , diff_angle)

        #         z_output = self.z_pid.update( heading_error )

        #         #print(heading_error , z_output)
        #     else:
        #         z_output = 0.0

        #     #print(diff_angle)
        #     #print(z_output)
        #    # rospy.loginfo(self.yaw )

           

        #     if self.yaw != None:
        #         #rospy.loginfo(self.yaw - diff_angle)

        #         if heading_error < 10 :
        #             z_output = 0
        #         if heading_rotation == 0: # 시계방향
        #             z_output = z_output
        #         elif heading_rotation == 1: # 반시계방향
        #             z_output = -z_output


        #         #print(z_output)
        #         # if self.yaw - diff_angle < 0:
        #         #     z_output = -0.5                
        #         # elif self.yaw - diff_angle > 5:
        #         #    z_output = 0.3
        #         # else:
        #         #    z_output = 0.0

        #     #print(z_output)
        #     #print(x_output , y_output)

            # end comment

            # Publish Velocity
            x_output = 1.0
            y_output = 0.0
            angular_z = 0.0

            vel_msg = Twist()
            vel_msg.linear.x = x_output
            vel_msg.linear.y = y_output
            # vel_msg.linear.x = 0.0
            # vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = angular_z
            
            self.vel_pub.publish(vel_msg)
                # 파일 닫기
            # f.close()
            # f1.close()

            rate.sleep()

    # tracking 관련 함수
    def distance_on_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 지구의 반지름 (m)
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        dx = R * math.cos(lat1_rad) * (lon2_rad - lon1_rad)
        dy = R * (lat2_rad - lat1_rad)
        return dx, dy
   
    # get target GPS
    def getHtml(self):
     
     
        while(True):
           
 
            response = requests.get("http://hajin220.dothome.co.kr/get_gps.php")
           
            _json = response.json()

            #print(_json)
            self.target_lat = float(_json['results'][0]['g_lat'])
            self.target_lng = float(_json['results'][0]['g_lng'])
           
            #print(float(self.lat))
            #print(float(self.lng))        
         
            time.sleep(0.01)

                # 파일에 텍스트 쓰기

    def calculate_bearing(self,pointA, pointB):
        """
        두 지점 사이의 이동 방향(베어링)을 계산합니다.
        pointA와 pointB는 (latitude, longitude) 형식의 튜플이어야 합니다.
        반환값은 방위각(0 ~ 360도)입니다.
        """

        lat1, lon1 = pointA
        lat2, lon2 = pointB

        # 위도와 경도를 라디안 단위로 변환
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # 경도 차이 계산
        dLon = lon2 - lon1

        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

        # 방위각 계산
        bearing = math.atan2(y, x)

        # 라디안 값을 도 단위로 변환
        bearing = math.degrees(bearing)

        # 0 ~ 360도 사이로 조정
        bearing = (bearing + 360) % 360

        return bearing
   
   
    def calculate_destination_coordinate(self, start_point, bearing, distance):
        """
        시작 지점과 이동 방향(베어링)을 기반으로 지정한 거리만큼 떨어진 새로운 좌표를 계산합니다.
        start_point: (latitude, longitude) 형식의 시작 지점 좌표
        bearing: 이동 방향(베어링)
        distance: 시작 지점에서 이동할 거리 (단위: 미터)
        반환값: (latitude, longitude) 형식의 새로운 좌표
        """

        # 시작 지점의 위도와 경도 추출
        lat1, lon1 = start_point

        # 이동 거리를 이동 방향의 벡터로 변환
        dx = distance * math.cos(math.radians(bearing))
        dy = distance * math.sin(math.radians(bearing))

        # 새로운 좌표 계산
        lat2 = lat1 + (dx / 111111)  # 1도의 위도 차이는 약 111,111 미터
        lon2 = lon1 + (dy / (111111 * math.cos(math.radians(lat1))))  # 1도의 경도 차이는 약 111,111 * cos(위도) 미터

        return lat2, lon2

if __name__ == "__main__":

    move_drone = MoveDrone()

     # Arm Drone
    #move_drone.arm()

    # Takeoff Drone
    #move_drone.takeoff()
    #rospy.sleep(2.0)

    move_drone.move_drone()
