#!/usr/bin/env python3
import rospy
import math
import dubins
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix

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

# Dubins 경로 생성 함수
def get_dubins_path(start, end, turning_radius):
    # 시작 및 끝 좌표에 방향을 추가합니다 (각도 단위: 라디안)
    start_with_angle = (start[0], start[1], 0)
    end_with_angle = (end[0], end[1], 0)
    configurations = dubins.shortest_path(start_with_angle, end_with_angle, turning_radius)
    return configurations.sample_many(1.0)

# GPS 좌표를 라디안으로 변환하는 함수
def gps_to_rad(lat, lon):
    return (math.radians(lat), math.radians(lon))

# 라디안을 GPS 좌표로 변환하는 함수
def rad_to_gps(lat, lon):
    return (math.degrees(lat), math.degrees(lon))

# 목표 GPS 좌표 설정
waypoints = [
     (47.4060880, 8.5525453),  # 시작 좌표
    (47.4079917, 8.5525833),  # 중간 좌표
    (47.4079660, 8.5558650)   # 목표 좌표
]

# Dubins 경로 생성
turning_radius = 0.002  # 약 200m 반경
paths = []
for i in range(len(waypoints) - 1):
    start = gps_to_rad(*waypoints[i])
    end = gps_to_rad(*waypoints[i + 1])
    path = get_dubins_path(start, end, turning_radius)
    paths.extend(path[0])  # path[0]은 위치 리스트, path[1]은 경로 길이

# 오프보드 모드로 전환 및 드론 무장
offb_set_mode = SetMode()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBool()
arm_cmd.value = True

last_req = rospy.Time.now()

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

    # Dubins 경로를 따라 목표 위치로 이동
    for path in paths:
        lat, lon = rad_to_gps(path[0], path[1])
        target_gps = GeoPoseStamped()
        target_gps.pose.position.latitude = lat
        target_gps.pose.position.longitude = lon
        target_gps.pose.position.altitude = 500  # 고도 (예시)
        gps_pos_pub.publish(target_gps)
        rate.sleep()

    rate.sleep()
