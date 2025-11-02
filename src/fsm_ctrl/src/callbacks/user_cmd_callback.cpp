/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-02 11
 * @FilePath: /fsm_ctrl/src/callbacks/user_cmd_callback.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/callbacks/user_cmd_callback.hpp"

namespace fsm_cb
{
    Eigen::Vector3d user_cmd_fcu_pos;    
    Eigen::Quaterniond user_cmd_fcu_quat;
    Eigen::Vector3d user_cmd_fcu_euler;

    Eigen::Vector3d user_cmd_vio_pos;    
    Eigen::Quaterniond user_cmd_vio_quat;
    Eigen::Vector3d user_cmd_vio_euler;

    /**
     * @description: 
     * @param {geometry_msgs::PoseStamped::ConstPtr} &msg
     * @return {*}
     */
    void UserCmdFcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        user_cmd_fcu_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        user_cmd_fcu_quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        user_cmd_fcu_euler = fsm_ut::QuatToEuler(user_cmd_fcu_quat);
    }

    /**
     * @description: 
     * @param {geometry_msgs::PoseStamped::ConstPtr} &msg
     * @return {*}
     */
    void UserCmdVioPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        user_cmd_vio_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        user_cmd_vio_quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        user_cmd_vio_euler = fsm_ut::QuatToEuler(user_cmd_vio_quat);
    }
} // namespace fsm_cb