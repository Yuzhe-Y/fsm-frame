/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-02 11
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/callbacks/user_cmd_callback.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __USER_CMD_CALLBACK_H__
#define __USER_CMD_CALLBACK_H__

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

namespace fsm_cb
{
    extern Eigen::Vector3d user_cmd_fcu_pos;    
    extern Eigen::Quaterniond user_cmd_fcu_quat;
    extern Eigen::Vector3d user_cmd_fcu_euler;

    extern Eigen::Vector3d user_cmd_vio_pos;    
    extern Eigen::Quaterniond user_cmd_vio_quat;
    extern Eigen::Vector3d user_cmd_vio_euler;

    void UserCmdFcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UserCmdVioPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
}

#endif // __USER_CMD_CALLBACK_H__
