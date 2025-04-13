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
    (47.4083660, 8.5521229),
    (47.4079933, 8.5514105),
    (47.4079660, 8.5558650),
    (47.4080189, 8.5564586),
]

# Dubins 경로 생성
turning_radius = 0.002  # 약 200m 반경
paths = []
for i in range(len(waypoints) - 1):
    start = gps_to_rad(*waypoints[i])
    end = gps_to_rad(*waypoints[i + 1])
    path = get_dubins_path(start, end, turning_radius)
    paths.extend(path[0])

# 오프보드 모드로 전환 및 드론 무장
offb_set_mode = SetMode()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBool()
arm_cmd.value = True

last_req = rospy.Time.now()

def distance(lat1, lon1, lat2, lon2):
    """Haversine formula to calculate distance between two GPS points."""
    R = 6371000  # Radius of the Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

current_waypoint_index = 0
waypoint_tolerance = 5.0  # meters

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

    # 현재 경로 점을 가져옵니다.
    if current_waypoint_index < len(paths):
        path = paths[current_waypoint_index]
        lat, lon = rad_to_gps(path[0], path[1])
        target_gps = GeoPoseStamped()
        target_gps.pose.position.latitude = lat
        target_gps.pose.position.longitude = lon
        target_gps.pose.position.altitude = 500  # 고도 (예시)
        gps_pos_pub.publish(target_gps)

        # 현재 위치와 경로 점 사이의 거리 계산
        dist_to_waypoint = distance(current_gps.latitude, current_gps.longitude, lat, lon)
        if dist_to_waypoint < waypoint_tolerance:
            rospy.loginfo(f"Waypoint {current_waypoint_index} reached.")
            current_waypoint_index += 1
    else:
        rospy.loginfo("All waypoints reached.")
        target_reached = True

    rate.sleep()
