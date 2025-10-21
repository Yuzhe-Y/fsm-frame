/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-06 20
 * @FilePath: /fsm_ctrl/src/callbacks/simulator_callback.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/callbacks/simulator_callback.hpp"

namespace fsm_cb
{
    Eigen::Vector3d simulator_pos;
    Eigen::Vector3d simulator_vel;
    Eigen::Vector3d simulator_acc;
    Eigen::Quaterniond simulator_quat;
    Eigen::Vector3d simulator_euler; 
    Eigen::Vector3d simulator_angular_vel;
    Eigen::Vector3d simulator_angular_acc;

    /**
     * @description: 
     * @param {nav_msgs::Odometry::ConstPtr} &msg
     * @return {*}
     */
    void SimulatorStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        simulator_pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        simulator_quat = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        simulator_euler = fsm_ut::QuatToEuler(simulator_quat);

        simulator_vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        simulator_angular_vel = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    }
}