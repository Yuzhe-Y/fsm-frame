/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-25 16
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/utils/communication_utils/px4_command.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _PX4_COMMAND_HPP_
#define _PX4_COMMAND_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <geometry_msgs/PoseStamped.h>

#include "fsm_ctrl/callbacks/external_odom_callback.hpp"

namespace fsm_ut
{
    mavros_msgs::PositionTarget SetTargetPositionAndYawCmd(double x, double y, double z, double yaw);
    mavros_msgs::PositionTarget SetTargetVelocityAndYawRateCmd(double x_vel, double y_vel, double z_vel, double yaw_rate);
    mavros_msgs::AttitudeTarget SetTargetRateAndTotalThrustCmd(double x_rate, double y_rate, double z_rate, double total_thrust);
    mavros_msgs::AttitudeTarget SetTargetAttAndTotalThrustCmd(double x, double y, double z, double w, double total_thrust);
    geometry_msgs::PoseStamped SetEkfExtPoseData(Eigen::Vector3d pos, Eigen::Quaterniond quat);
    
    void InitPX4(mavros_msgs::SetMode& offboard_mode,
                 mavros_msgs::SetMode& land_mode,
                 mavros_msgs::CommandBool& arm_cmd,
                 mavros_msgs::CommandBool& disarm_cmd,
                 ros::Publisher& local_setpoint_position_publisher,
                 ros::Rate& rate);
    void CheckAndSwitchToOffboardAndArm(const mavros_msgs::State& current_state,
                                        mavros_msgs::SetMode& offboard_mode,
                                        mavros_msgs::CommandBool& arm_cmd,
                                        ros::ServiceClient& set_mode_client,
                                        ros::ServiceClient& arming_cmd_client,
                                        ros::Time& last_request);

} // namespace fsm_ut
#endif