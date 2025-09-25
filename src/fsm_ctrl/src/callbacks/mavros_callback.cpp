/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-24 21
 * @FilePath: /fsm_ctrl/src/callbacks/mavros_callback.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/callbacks/mavros_callback.hpp"

namespace fsm_cb
{
    bool mavros_ekf_ready;

    sensor_msgs::BatteryState mavros_battery;
    mavros_msgs::State mavros_state;
    Eigen::Vector3d mavros_fcu_pos;    
    Eigen::Vector3d mavros_fcu_vel;     
    Eigen::Quaterniond mavros_fcu_quat;
    Eigen::Vector3d mavros_fcu_euler;

    Eigen::Vector3d mavros_imu_acc;
    Eigen::Vector3d mavros_imu_rate;
    mavros_msgs::RCIn mavros_rc;
    bool is_quat_init;
    bool is_need_rot;

    /**
     * @description: 
     * @param {std_msgs::Bool::ConstPtr} &msg
     * @return {*}
     */
    void MavrosEkfReadyCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        mavros_ekf_ready = msg->data;
    }

    /**
     * @description: 
     * @param {sensor_msgs::BatteryState::ConstPtr} &msg
     * @return {*}
     */
    void MavrosBatteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        mavros_battery = *msg;
    }

    /**
     * @description: 
     * @param {geometry_msgs::PoseStamped::ConstPtr} &msg
     * @return {*}
     */
    void MavrosFcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        mavros_fcu_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        mavros_fcu_quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

        if(!is_quat_init)
        {
            is_quat_init = true;
            if(mavros_fcu_quat.w() < 0.0) 
            {
                is_need_rot = true;
            }
            else 
            {
                is_need_rot = false;
            }
        }
        if(is_need_rot) 
        {
            mavros_fcu_quat = mavros_fcu_quat * Eigen::Quaterniond(-1.0, 0.0, 0.0, 0.0);
        }

        mavros_fcu_euler = fsm_ut::QuatToEuler(mavros_fcu_quat);
    }

    /**
     * @description: 
     * @param {geometry_msgs::TwistStamped::ConstPtr} &msg
     * @return {*}
     */
    void MavrosFcuVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        mavros_fcu_vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    }

    /**
     * @description: 
     * @param {sensor_msgs::Imu::ConstPtr} &msg
     * @return {*}
     */
    void MavrosImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        mavros_imu_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        mavros_imu_rate = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);   
    }

    /**
     * @description: 
     * @param {mavros_msgs::RCIn::ConstPtr} &msg
     * @return {*}
     */
    void MavrosRcCallback(const mavros_msgs::RCIn::ConstPtr &msg)
    {
        mavros_rc = *msg;
    }

    void MavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        mavros_state = *msg;
    }
} // namespace fsm_ctrl_mavros_callbacks