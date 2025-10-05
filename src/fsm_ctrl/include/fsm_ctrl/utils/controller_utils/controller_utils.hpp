/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-05 15
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/utils/controller_utils/controller_utils.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _CONTROLLER_UTILS_HPP_
#define _CONTROLLER_UTILS_HPP_

#include <ros/ros.h>
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "fsm_ctrl/controllers/ipopt_controller/nmpc_controller.hpp"
#include "fsm_ctrl/controllers/dfbc_controller/dfbc_controller.hpp"
#include "fsm_ctrl/callbacks/mavros_callback.hpp"

#include "const_params.h"

namespace fsm_ut
{

    struct Controller
    {
        bool use_defalut_controller; //是否使用默认控制器
        int defalut_controller_type; //默认控制器类型
        int nmpc_controller_type;

        Eigen::Vector3d position_fdb; //位置反馈
        Eigen::Vector3d velocity_fdb; //速度反馈
        Eigen::Quaterniond attitude_fdb; //姿态反馈
        Eigen::Vector3d angular_velocity_fdb; //角速度反馈
        Eigen::Vector4d motor_rpm_fdb; //电机转速反馈

        std::vector<Eigen::Vector3d> position_ref; //位置参考
        std::vector<Eigen::Vector3d> velocity_ref; //速度参考
        std::vector<Eigen::Vector3d> acceleration_ref; //加速度参考
        std::vector<Eigen::Quaterniond> attitude_ref; //姿态参考
        std::vector<Eigen::Vector3d> angular_velocity_ref; //角速度参考
        std::vector<double> acc_z_ref;
        std::vector<Eigen::Vector4d> force_ref; //电机
        std::vector<Eigen::Vector4d> delta_force_ref; //电机增量
    };

    void SetControllerFdb(Controller& controller);

    void IpoptNmpcWandTotalFControllerInit(ros::NodeHandle& nh, NMPC_Ctrller_simple& nmpc_controller);
    void DFBCControllerInit(ros::NodeHandle& nh, DFBC_Controller& dfbc_controller);

} // namespace fsm_ut
#endif