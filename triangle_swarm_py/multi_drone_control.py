#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import math
import time

N_DRONES = 5  # 드론 개수
DRONE_STATES = [State() for _ in range(N_DRONES)]  # 드론 상태
ARM_SERVICES = []  # 드론 시동 서비스
SET_MODE_SERVICES = []  # 드론 모드 설정 서비스

def state_cb(data, drone_id):
    global DRONE_STATES
    DRONE_STATES[drone_id] = data

def initialize_drones():
    global ARM_SERVICES, SET_MODE_SERVICES
    for i in range(N_DRONES):
        rospy.Subscriber(f"uav{i}/mavros/state", State, state_cb, i)
        ARM_SERVICES.append(rospy.ServiceProxy(f"uav{i}/mavros/cmd/arming", CommandBool))
        SET_MODE_SERVICES.append(rospy.ServiceProxy(f"uav{i}/mavros/set_mode", SetMode))

def form_triangle(leader_pose):
    follower_positions = []
    
    angle_offset = 180.0 / (N_DRONES - 1)
    
    for i in range(1, N_DRONES):
        if i % 2 == 0:
            angle = -angle_offset * (i // 2)
        else:
            angle = angle_offset * ((i + 1) // 2)
        
        x = leader_pose.pose.position.x + 5 * math.cos(math.radians(angle))
        y = leader_pose.pose.position.y + 5 * math.sin(math.radians(angle))
        z = leader_pose.pose.position.z  # 팔로워 드론의 고도는 리더 드론과 동일

        follower_pose = PoseStamped()
        follower_pose.pose.position.x = x
        follower_pose.pose.position.y = y
        follower_pose.pose.position.z = z
        follower_positions.append(follower_pose)
    
    return follower_positions

def main():
    rospy.init_node("multi_drone_control", anonymous=True)
    initialize_drones()

    leader_pose_sub = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, state_cb, 0)

    follower_pose_pubs = [rospy.Publisher(f"uav{i}/mavros/setpoint_position/local", PoseStamped, queue_size=10) for i in range(1, N_DRONES)]

    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        leader_pose = rospy.wait_for_message("uav0/mavros/local_position/pose", PoseStamped)

        if leader_pose.pose.position.z >= 2.0:
            for i in range(1, N_DRONES):
                if not DRONE_STATES[i].armed:
                    ARM_SERVICES[i](True)
                    SET_MODE_SERVICES[i](0, "OFFBOARD")
                    time.sleep(1)

        follower_positions = form_triangle(leader_pose)

        for i in range(N_DRONES - 1):
            if DRONE_STATES[i+1].connected:
                follower_pose_pubs[i].publish(follower_positions[i])

        rate.sleep()

if __name__ == "__main__":
    main()
