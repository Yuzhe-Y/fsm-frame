/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-07-15 20
 * @FilePath: /fsm_ctrl/src/utils/communication_utils/px4_command.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */

#include "fsm_ctrl/utils/communication_utils/px4_command.hpp"

/**
 * @description: Set px4 target position and yaw command
 * @param {double} x
 * @param {double} y
 * @param {double} z
 * @param {double} yaw
 * @return {mavros_msgs::PositionTarget} msg
 */
mavros_msgs::PositionTarget SetTargetPositionAndYawCmd(double x, double y, double z, double yaw)
{
    mavros_msgs::PositionTarget msg;

    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.yaw = yaw;
    
    return msg;
}

/**
 * @description: Set px4 target velocity and yaw rate command
 * @param {double} x_vel
 * @param {double} y_vel
 * @param {double} z_vel
 * @param {double} yaw_rate
 * @return {mavros_msgs::PositionTarget} msg
 */
mavros_msgs::PositionTarget SetTargetVelocityAndYawRateCmd(double x_vel, double y_vel, double z_vel, double yaw_rate)
{
    mavros_msgs::PositionTarget msg;

    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    msg.velocity.x = x_vel;
    msg.velocity.y = y_vel;
    msg.velocity.z = z_vel;
    msg.yaw_rate = yaw_rate;
    
    return msg;
}

/**
 * @description: Set px4 target body rate and total thrust command
 * @param {double} x_rate
 * @param {double} y_rate
 * @param {double} z_rate
 * @param {double} total_thrust
 * @return {mavros_msgs::AttitudeTarget} msg
 */
mavros_msgs::AttitudeTarget SetTargetRateAndTotalThrustCmd(double x_rate, double y_rate, double z_rate, double total_thrust)
{
    mavros_msgs::AttitudeTarget msg;

    msg.header.frame_id = std::string("FCU");
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    
    msg.body_rate.x = x_rate;
    msg.body_rate.y = y_rate;
    msg.body_rate.z = z_rate;
    msg.thrust = total_thrust;

    return msg;
}

/**
 * @description: Set px4 target attitude and total thrust command
 * @param {double} x
 * @param {double} y
 * @param {double} z
 * @param {double} w
 * @param {double} total_thrust
 * @return {mavros_msgs::AttitudeTarget} msg
 */
mavros_msgs::AttitudeTarget SetTargetAttAndTotalThrustCmd(double x, double y, double z, double w, double total_thrust)
{
    mavros_msgs::AttitudeTarget msg;

    msg.header.frame_id = std::string("FCU");
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    msg.orientation.x = x;
    msg.orientation.y = y;
    msg.orientation.z = z;
    msg.orientation.w = w;
    msg.thrust = total_thrust;

    return msg;
}

/**
 * @description: 
 * @param {Vector3d} pos
 * @param {Quaterniond} quat
 * @return {geometry_msgs::PoseStamped} msg
 */
geometry_msgs::PoseStamped SetEkfExtPoseData(Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
    geometry_msgs::PoseStamped msg;

    msg.header.stamp = fsm_cb::ext_odom_stamp;

    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();

    return msg;
}

/**
 * @description: Initialize PX4, build communication
 * @param {mavros_msgs::SetMode&} offboard_mode
 * @param {mavros_msgs::SetMode&} land_mode
 * @param {mavros_msgs::CommandBool&} arm_cmd
 * @param {mavros_msgs::CommandBool&} disarm_cmd
 * @param {ros::Publisher&} local_setpoint_position_publisher
 * @param {ros::Rate&} rate
 * @return {*}
 */
void InitPX4(mavros_msgs::SetMode& offboard_mode,
             mavros_msgs::SetMode& land_mode,
             mavros_msgs::CommandBool& arm_cmd,
             mavros_msgs::CommandBool& disarm_cmd,
             ros::Publisher& local_setpoint_position_publisher,
             ros::Rate& rate)
{
    offboard_mode.request.custom_mode = "OFFBOARD";
    land_mode.request.custom_mode = "LAND";
    arm_cmd.request.value = false;
    disarm_cmd.request.value = false;
    
    geometry_msgs::PoseStamped pos_cmd;
    pos_cmd.pose.position.x = 0.0;
    pos_cmd.pose.position.y = 0.0;
    pos_cmd.pose.position.z = 0.5;
    
    for (int i = 100; ros::ok() && i > 0; --i)
    {
         local_setpoint_position_publisher.publish(pos_cmd);
         ros::spinOnce();
         rate.sleep();
    }
}

/**
 * @description: Send command to switch to OFFBOARD mode and arm the vehicle
 * @param {mavros_msgs::State} current_state
 * @param {mavros_msgs::SetMode&} offboard_mode
 * @param {mavros_msgs::CommandBool&} arm_cmd
 * @param {ros::ServiceClient&} set_mode_client
 * @param {ros::ServiceClient&} arming_cmd_client
 * @param {ros::Time&} last_request
 * @return {*}
 */
void CheckAndSwitchToOffboardAndArm(const mavros_msgs::State& current_state,
                                    mavros_msgs::SetMode& offboard_mode,
                                    mavros_msgs::CommandBool& arm_cmd,
                                    ros::ServiceClient& set_mode_client,
                                    ros::ServiceClient& arming_cmd_client,
                                    ros::Time& last_request)
{
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
        if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
        {
            ROS_WARN("Mode switched to OFFBOARD");
        }
        last_request = ros::Time::now();
    }
    else
    {
        if (!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_cmd_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_WARN("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
}
