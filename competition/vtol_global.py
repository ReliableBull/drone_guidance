#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

current_state = State()
current_gps = NavSatFix()

def state_cb(state):
    global current_state
    current_state = state

def gps_cb(gps):
    global current_gps
    current_gps = gps

rospy.init_node('offb_node', anonymous=True)

state_sub = rospy.Subscriber('mavros/state', State, state_cb)
gps_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, gps_cb)
gps_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

rospy.wait_for_service('mavros/cmd/arming')
arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

rate = rospy.Rate(20)

# 초기 목표 GPS 위치 설정 (현재 위치에서 시작)
target_gps = GeoPoseStamped()
target_gps.pose.position.latitude = 47.4026272  # 예시 목표 좌표
target_gps.pose.position.longitude = 8.5597013  # 예시 목표 좌표
target_gps.pose.position.altitude = 500  # 목표 고도 (예시)

# 오프보드 모드로 전환 및 드론 무장
offb_set_mode = SetMode()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBool()
arm_cmd.value = True

last_req = rospy.Time.now()

while not rospy.is_shutdown():
    gps_pos_pub.publish(target_gps)
    rate.sleep()
