#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

mavros_msgs::State uav0_current_state;
mavros_msgs::State uav1_current_state;
mavros_msgs::State uav2_current_state;
mavros_msgs::State uav3_current_state;
mavros_msgs::State uav4_current_state;
mavros_msgs::State uav5_current_state;
mavros_msgs::State uav6_current_state;
mavros_msgs::State uav7_current_state;
mavros_msgs::State uav8_current_state;
mavros_msgs::State uav9_current_state;

geometry_msgs::PoseStamped leader_pose;

double leader_heading = 0.0;

void uav0_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav0_current_state = *msg;
}

void uav1_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}

void uav2_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

void uav3_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav3_current_state = *msg;
}

void uav4_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav4_current_state = *msg;
}

void uav5_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav5_current_state = *msg;
}

void uav6_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav6_current_state = *msg;
}

void uav7_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav7_current_state = *msg;
}

void uav8_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav8_current_state = *msg;
}

void uav9_state_now(const mavros_msgs::State::ConstPtr& msg){
    uav9_current_state = *msg;
}

void goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    leader_pose = *msg;
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setX(leader_pose.pose.orientation.x);
    tf_quaternion.setY(leader_pose.pose.orientation.y);
    tf_quaternion.setZ(leader_pose.pose.orientation.z);
    tf_quaternion.setW(leader_pose.pose.orientation.w);

    tf2::Matrix3x3 mat(tf_quaternion);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    leader_heading = yaw;
}

bool setOffboardMode(ros::ServiceClient& set_mode_client) {
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
        return true;
    }
    return false;
}

bool armVehicle(ros::ServiceClient& arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        return true;
    }
    return false;
}

bool setReturnMode(ros::ServiceClient& set_mode_client) {
    mavros_msgs::SetMode return_set_mode;
    return_set_mode.request.custom_mode = "AUTO.RTL";
    if (set_mode_client.call(return_set_mode) && return_set_mode.response.mode_sent) {
        return true;
    }
    return false;
}

void swarm_setpoint(ros::Publisher& local_pos_pub, ros::Rate& rate, double offset_x, double offset_y, double offset_z, double min_height) {
    geometry_msgs::PoseStamped pose;

    double l = sqrt(pow(offset_x, 2) + pow(offset_y, 2)); 

    double radian = atan2(offset_y, offset_x);

    pose.pose.position.x = leader_pose.pose.position.x + (l * sin(radian - leader_heading));
    pose.pose.position.y = leader_pose.pose.position.y + (l * cos(radian - leader_heading));
    pose.pose.position.z = leader_pose.pose.position.z + offset_z;
    pose.pose.orientation.x = leader_pose.pose.orientation.x;
    pose.pose.orientation.y = leader_pose.pose.orientation.y;
    pose.pose.orientation.z = leader_pose.pose.orientation.z;
    pose.pose.orientation.w = leader_pose.pose.orientation.w;

    // ROS_INFO("leader_heading = %f", leader_heading);
    

    if (leader_pose.pose.position.z >= min_height) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_voxl2");
    ros::NodeHandle nh;

    ros::Subscriber leader_pos_sub = nh.subscribe<geometry_msgs::PoseStamped> ("uav0/mavros1/local_position/pose", 10, goal_pose);
    
    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State> ("uav0/mavros1/state", 10, uav0_state_now);
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State> ("uav1/mavros1/state", 10, uav1_state_now);
    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State> ("uav2/mavros1/state", 10, uav2_state_now);
    ros::Subscriber uav3_state_sub = nh.subscribe<mavros_msgs::State> ("uav3/mavros1/state", 10, uav3_state_now);
    ros::Subscriber uav4_state_sub = nh.subscribe<mavros_msgs::State> ("uav4/mavros1/state", 10, uav4_state_now);
    ros::Subscriber uav5_state_sub = nh.subscribe<mavros_msgs::State> ("uav5/mavros1/state", 10, uav5_state_now);
    ros::Subscriber uav6_state_sub = nh.subscribe<mavros_msgs::State> ("uav6/mavros1/state", 10, uav6_state_now);
    ros::Subscriber uav7_state_sub = nh.subscribe<mavros_msgs::State> ("uav7/mavros1/state", 10, uav7_state_now);
    ros::Subscriber uav8_state_sub = nh.subscribe<mavros_msgs::State> ("uav8/mavros1/state", 10, uav8_state_now);
    ros::Subscriber uav9_state_sub = nh.subscribe<mavros_msgs::State> ("uav9/mavros1/state", 10, uav9_state_now);
    
    ros::Publisher uav1_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav1/mavros1/setpoint_position/local", 10);
    ros::Publisher uav2_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav2/mavros1/setpoint_position/local", 10);
    ros::Publisher uav3_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav3/mavros1/setpoint_position/local", 10);
    ros::Publisher uav4_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav4/mavros1/setpoint_position/local", 10);
    ros::Publisher uav5_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav5/mavros1/setpoint_position/local", 10);
    ros::Publisher uav6_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav6/mavros1/setpoint_position/local", 10);
    ros::Publisher uav7_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav7/mavros1/setpoint_position/local", 10);
    ros::Publisher uav8_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav8/mavros1/setpoint_position/local", 10);
    ros::Publisher uav9_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("uav9/mavros1/setpoint_position/local", 10);

    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav1/mavros1/cmd/arming");
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav2/mavros1/cmd/arming");
    ros::ServiceClient uav3_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav3/mavros1/cmd/arming");
    ros::ServiceClient uav4_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav4/mavros1/cmd/arming");
    ros::ServiceClient uav5_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav5/mavros1/cmd/arming");
    ros::ServiceClient uav6_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav6/mavros1/cmd/arming");
    ros::ServiceClient uav7_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav7/mavros1/cmd/arming");
    ros::ServiceClient uav8_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav8/mavros1/cmd/arming");
    ros::ServiceClient uav9_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("uav9/mavros1/cmd/arming");

    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav1/mavros1/set_mode");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav2/mavros1/set_mode");
    ros::ServiceClient uav3_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav3/mavros1/set_mode");
    ros::ServiceClient uav4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav4/mavros1/set_mode");
    ros::ServiceClient uav5_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav5/mavros1/set_mode");
    ros::ServiceClient uav6_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav6/mavros1/set_mode");
    ros::ServiceClient uav7_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav7/mavros1/set_mode");
    ros::ServiceClient uav8_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav8/mavros1/set_mode");
    ros::ServiceClient uav9_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("uav9/mavros1/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        if (ros::ok() && uav1_current_state.connected) {
            if (uav0_current_state.mode == "AUTO.RTL") {
                setReturnMode(uav1_set_mode_client);
            }else{
                if( uav1_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (setOffboardMode(uav1_set_mode_client)) {
                        ROS_INFO("uav1_Offboard enabled");
                        last_request = ros::Time::now();
                    }
                } else {
                    if( !uav1_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                        if (armVehicle(uav1_arming_client)) {
                            ROS_INFO("uav1_Vehicle armed");
                            last_request = ros::Time::now();
                        }
                    }
                }
                swarm_setpoint(uav1_pos_pub, rate, -5, -5, 1, 2);
                ros::spinOnce();
                rate.sleep();  
            }
            
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav2_current_state.connected) {
            if (uav0_current_state.mode == "AUTO.RTL") {
                setReturnMode(uav2_set_mode_client);
            } else{
                if( uav2_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (setOffboardMode(uav2_set_mode_client)) {
                        ROS_INFO("uav2_Offboard enabled");
                        last_request = ros::Time::now();
                    }
                    } else {
                        if( !uav2_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                            if (armVehicle(uav2_arming_client)) {
                                ROS_INFO("uav2_Vehicle armed");
                                last_request = ros::Time::now();
                            }
                        }
                    }
                swarm_setpoint(uav2_pos_pub, rate,  5, -5, 1, 2);
                ros::spinOnce();
                rate.sleep();
            }
            
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav3_current_state.connected) {
            if (uav0_current_state.mode == "AUTO.RTL") {
                setReturnMode(uav2_set_mode_client);
            } else{
                if( uav3_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (setOffboardMode(uav3_set_mode_client)) {
                        ROS_INFO("uav3_Offboard enabled");
                        last_request = ros::Time::now();
                    }
                } else {
                    if( !uav3_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                        if (armVehicle(uav3_arming_client)) {
                            ROS_INFO("uav3_Vehicle armed");
                            last_request = ros::Time::now();
                        }
                    }
                }
                swarm_setpoint(uav3_pos_pub, rate,  -10, -10, 1, 2);
                ros::spinOnce();
                rate.sleep();
            }
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav4_current_state.connected) {
            if( uav4_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav4_set_mode_client)) {
                    ROS_INFO("uav4_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav4_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav4_arming_client)) {
                        ROS_INFO("uav4_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav4_pos_pub, rate,  10, -10, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav5_current_state.connected) {
            if( uav5_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav5_set_mode_client)) {
                    ROS_INFO("uav5_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav5_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav5_arming_client)) {
                        ROS_INFO("uav5_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav5_pos_pub, rate, -20, -20, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav6_current_state.connected) {
            if( uav6_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav6_set_mode_client)) {
                    ROS_INFO("uav6_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav6_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav6_arming_client)) {
                        ROS_INFO("uav6_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav6_pos_pub, rate,  20, -20, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav7_current_state.connected) {
            if( uav7_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav7_set_mode_client)) {
                    ROS_INFO("uav7_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav7_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav7_arming_client)) {
                        ROS_INFO("uav7_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav7_pos_pub, rate, -30, -30, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav8_current_state.connected) {
            if( uav8_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav8_set_mode_client)) {
                    ROS_INFO("uav8_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav8_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav8_arming_client)) {
                        ROS_INFO("uav8_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav8_pos_pub, rate,  30, -30, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

        if (ros::ok() && uav9_current_state.connected) {
            if( uav9_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                if (setOffboardMode(uav9_set_mode_client)) {
                    ROS_INFO("uav9_Offboard enabled");
                    last_request = ros::Time::now();
                }
            } else {
                if( !uav9_current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if (armVehicle(uav9_arming_client)) {
                        ROS_INFO("uav9_Vehicle armed");
                        last_request = ros::Time::now();
                    }
                }
            }
            swarm_setpoint(uav9_pos_pub, rate,  30, -30, 1, 2);
            ros::spinOnce();
            rate.sleep();
        }else {
            ros::spinOnce();
            rate.sleep();
        }

    }
    return 0;
}
