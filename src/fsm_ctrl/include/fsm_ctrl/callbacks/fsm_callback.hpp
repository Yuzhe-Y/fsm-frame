/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-02 10
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/callbacks/fsm_callback.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __MAVROS_CALLBACK_H__
#define __MAVROS_CALLBACK_H__

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ESCStatus.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

namespace fsm_cb
{
    extern bool is_ekf_converge;

    extern sensor_msgs::BatteryState mavros_battery;
    extern mavros_msgs::State mavros_state;

    extern Eigen::Vector3d mavros_fcu_pos;    
    extern Eigen::Vector3d mavros_fcu_vel;     
    extern Eigen::Quaterniond mavros_fcu_quat;
    extern Eigen::Vector3d mavros_fcu_euler;
    
    extern Eigen::Vector3d mavros_imu_acc;
    extern Eigen::Vector3d mavros_imu_rate;
    extern Eigen::Quaterniond mavros_imu_quat;
    extern Eigen::Vector3d mavros_imu_euler;

    extern double mavros_esc_voltage;
    extern Eigen::Vector4d mavros_esc_rotor_speed;

    extern mavros_msgs::RCIn mavros_rc;

    extern bool is_quat_init;         //是否收到姿态反馈（初始化）
    extern bool is_need_rot;          //检测第一帧飞控反馈四元数qw是否为-1，是为1，不是为0，如果是则执行四元数反转工作


    void MavrosFusionCallback(const std_msgs::Bool::ConstPtr &msg);

    void MavrosBatteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void MavrosFcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void MavrosFcuVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void MavrosImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void MavrosEscCallback(const mavros_msgs::ESCStatus::ConstPtr &msg);
    void MavrosRcCallback(const mavros_msgs::RCIn::ConstPtr &msg);
    void MavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
}

#endif // __MAVROS_CALLBACK_H__
