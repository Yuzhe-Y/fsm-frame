/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-07-15 18
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/callbacks/external_odom_callback.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __EXTERNAL_ODOM_CALLBACK_H__
#define __EXTERNAL_ODOM_CALLBACK_H__

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

namespace fsm_cb
{
    extern bool is_mocap_init;
    extern int mocap_frame;
    extern Eigen::Vector3d mocap_pos;                   
    extern Eigen::Quaterniond mocap_quat;
    extern Eigen::Vector3d mocap_euler;
    extern Eigen::Vector3d mocap_pos_init;                   
    extern Eigen::Quaterniond mocap_quat_init;

    extern Eigen::Vector3d lidar_pos;
    extern Eigen::Vector3d lidar_vel;                     
    extern Eigen::Quaterniond lidar_quat;
    extern Eigen::Vector3d lidar_euler; 

    extern Eigen::Vector3d camera_pos;
    extern Eigen::Vector3d camera_vel;                     
    extern Eigen::Quaterniond camera_quat;
    extern Eigen::Vector3d camera_euler;

    extern ros::Time ext_odom_stamp;
    extern bool is_source_new;

    void MocapOdomCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void LidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void CameraOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
}

#endif // __EXTERNAL_ODOM_CALLBACK_H__