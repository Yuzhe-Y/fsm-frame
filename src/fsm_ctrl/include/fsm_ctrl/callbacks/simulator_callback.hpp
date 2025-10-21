/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-06 20
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/callbacks/simulator_callback.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __SIMULATOR_CALLBACK_H__
#define __SIMULATOR_CALLBACK_H__

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <nav_msgs/Odometry.h>

#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

namespace fsm_cb
{
    extern Eigen::Vector3d simulator_pos;
    extern Eigen::Vector3d simulator_vel;
    extern Eigen::Vector3d simulator_acc;
    extern Eigen::Quaterniond simulator_quat;
    extern Eigen::Vector3d simulator_euler; 
    extern Eigen::Vector3d simulator_angular_vel;
    extern Eigen::Vector3d simulator_angular_acc;

    void SimulatorStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
}

#endif // __MAVROS_CALLBACK_H__