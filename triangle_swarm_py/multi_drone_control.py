#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

import math

# Global variables
leader_pose = PoseStamped()
follower_states = {
    "uav1": State(),
    "uav2": State(),
    "uav3": State(),
    "uav4": State(),
    "uav5": State(),
}
# Global variable to store leader's velocity
leader_velocity = None

# Callback for leader velocity
def leader_velocity_cb(data):
    global leader_velocity
    leader_velocity = data

# Callback for leader state
def leader_state_cb(data):
    global leader_pose
    leader_pose = data

# Callback for follower states
def follower_state_cb(uav_name, data):
    global follower_states
    follower_states[uav_name] = data

# Main function
def main():
    rospy.init_node('follow_the_leader', anonymous=True)
    rospy.loginfo("Node initialized")

    # Subscribe to leader's position
    rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, leader_state_cb)
    

    # Subscribe to follower states
    for uav_name in follower_states.keys():
        rospy.Subscriber("{}/mavros/state".format(uav_name), State, callback=lambda data, uav_name=uav_name: follower_state_cb(uav_name, data))

    # Service clients for setting mode and arming
    set_mode_srvs = {uav_name: rospy.ServiceProxy("{}/mavros/set_mode".format(uav_name), SetMode) for uav_name in follower_states.keys()}
    arming_srvs = {uav_name: rospy.ServiceProxy("{}/mavros/cmd/arming".format(uav_name), CommandBool) for uav_name in follower_states.keys()}

    # Publishers for followers' target positions
    local_pos_pubs = {uav_name: rospy.Publisher("{}/mavros/setpoint_position/local".format(uav_name), PoseStamped, queue_size=10) for uav_name in follower_states.keys()}

    rate = rospy.Rate(20)

    is_first_time = True  # to check if OFFBOARD and arm state set for the first time

    while not rospy.is_shutdown():

        if leader_pose.pose.position.z > 2:
            
            orientation_q = leader_pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (_, _, yaw) = euler_from_quaternion(orientation_list)

            for idx, uav_name in enumerate(sorted(follower_states.keys())):

                if follower_states[uav_name].mode != "OFFBOARD":
                    res_mode = set_mode_srvs[uav_name](custom_mode="OFFBOARD")
                    res_arm = arming_srvs[uav_name](True)
                    
                    if res_mode and res_arm:
                        rospy.loginfo("{} successfully changed to OFFBOARD and armed.".format(uav_name))
                    else:
                        rospy.loginfo("Failed to set OFFBOARD and arm {}".format(uav_name))

                x_offset, y_offset = 0, 0
                row = idx // 2
                col = idx % 2
                
                if row == 0:
                    x_offset = -3
                elif row == 1:
                    x_offset = -6
                else:
                    x_offset = -9
                
                if col == 0:
                    y_offset = -3
                else:
                    y_offset = 3

                x_rotated = x_offset * math.cos(yaw) - y_offset * math.sin(yaw)
                y_rotated = x_offset * math.sin(yaw) + y_offset * math.cos(yaw)

                follow_pose = PoseStamped()
                follow_pose.header.stamp = rospy.Time.now()
                follow_pose.pose.position.x = leader_pose.pose.position.x + x_rotated
                follow_pose.pose.position.y = leader_pose.pose.position.y + y_rotated
                follow_pose.pose.position.z = leader_pose.pose.position.z
                follow_pose.pose.orientation = orientation_q
                
                local_pos_pubs[uav_name].publish(follow_pose)

        rate.sleep()








if __name__ == '__main__':
    main()
