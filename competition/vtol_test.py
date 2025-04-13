#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped

def state_cb(state):
    global current_state
    current_state = state

print("12321")
current_state = State()
print("1")
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
print("2")
vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

print("3")
# rospy.wait_for_service('mavros/cmd/arming')
# print("4")
# arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
# print("5")
set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

print("CALL11")
rospy.init_node('offb_node', anonymous=True)


rate = rospy.Rate(20)
print("CALL11")
# while not current_state.connected:
#     rospy.sleep(1)

# 초기 속도 설정 (정지 상태)
velocity = TwistStamped()
velocity.twist.linear.x = 0
velocity.twist.linear.y = 0
velocity.twist.linear.z = 0

# # 100개의 초기 메시지를 보내어 오프보드 모드를 설정
# for i in range(100):
#     vel_pub.publish(velocity)
#     rate.sleep()

offb_set_mode = SetMode()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBool()
arm_cmd.value = True

last_req = rospy.Time.now()
print("CALL11")
while not rospy.is_shutdown():
    print("CALL")
    # 원하는 속도로 설정
    velocity.twist.linear.x = 0  # x 방향으로 5 m/s
    velocity.twist.linear.y = 1
    velocity.twist.linear.z = 0

    vel_pub.publish(velocity)
    rate.sleep()

rospy.loginfo("Switching to FIXED WING mode")

# 고정익 모드로 전환
fixed_wing_set_mode = SetMode()
fixed_wing_set_mode.custom_mode = 'AUTO.LOITER'

while not rospy.is_shutdown():
    if current_state.mode != 'AUTO.LOITER' and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
        if set_mode_service(0, fixed_wing_set_mode.custom_mode).mode_sent:
            rospy.loginfo("Fixed wing mode enabled")
        last_req = rospy.Time.now()

    # 원하는 속도로 설정 (고정익 모드에서는 x 방향 속도 유지)
    velocity.twist.linear.x = 20  # 고정익 모드에서는 속도를 더 높일 수 있음
    velocity.twist.linear.y = 0
    velocity.twist.linear.z = 0

    vel_pub.publish(velocity)
    rate.sleep()
