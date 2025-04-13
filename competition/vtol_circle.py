#!/usr/bin/env python3
import rospy
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geographic_msgs.msg import GeoPoseStamped

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

rospy.init_node('offb_node', anonymous=True)

state_sub = rospy.Subscriber('mavros/state', State, state_cb)
gps_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

rospy.wait_for_service('mavros/cmd/arming')
arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

rate = rospy.Rate(20)

# 중심 좌표 설정
center_lat = 47.4022224  # 예시 중심 좌표
center_lon = 8.5609224   # 예시 중심 좌표
radius = 0.0018  # 약 200m 반경

# 각도 증가 값 계산 (속도 15m/s, 반경 200m)
angle_increment = 0.075  # 약 0.075rad/s

# 오프보드 모드로 전환 및 드론 무장
offb_set_mode = SetMode()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBool()
arm_cmd.value = True

last_req = rospy.Time.now()

angle = 0.0  # 초기 각도

while not rospy.is_shutdown():
    if current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
        if set_mode_service(0, offb_set_mode.custom_mode).mode_sent:
            rospy.loginfo("Offboard mode enabled")
        last_req = rospy.Time.now()
    else:
        if not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if arm_service(arm_cmd).success:
                rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()

    # 원 경로를 따라 새로운 목표 위치 계산
    target_gps = GeoPoseStamped()
    target_gps.pose.position.latitude = center_lat + radius * math.cos(angle)
    target_gps.pose.position.longitude = center_lon + radius * math.sin(angle)
    target_gps.pose.position.altitude = 500  # 고도 (예시)

    gps_pos_pub.publish(target_gps)

    angle += angle_increment  # 각도 증가 (속도에 따라 조정 가능)

    if angle >= 2 * math.pi:
        angle -= 2 * math.pi  # 0으로 각도 초기화

    rate.sleep()
