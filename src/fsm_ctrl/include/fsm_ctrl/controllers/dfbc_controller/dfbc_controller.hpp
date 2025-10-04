/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-04 15
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/controllers/dfbc_controller/dfbc_controller.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _DFBC_CONTROLLER_HPP_
#define _DFBC_CONTROLLER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <mavros_msgs/AttitudeTarget.h>

#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

#include "const_params.h"

class DFBC_Controller
{
    public:
        Eigen::Matrix3d Kp, Ki, Kd, Kv;
        Eigen::Quaterniond quat_command;
        double thrust_percentage;

        fsm_ut::Thr_LSE thr_est;

        DFBC_Controller();
        ~DFBC_Controller() = default;
        void Init(double _ctrl_rate, double _hover_thr, const Eigen::Vector3d &kp, Eigen::Vector3d &ki, Eigen::Vector3d &kd, Eigen::Vector3d &kv);
        void AttCtrl(const Eigen::Vector3d &pos_ref, const Eigen::Vector3d &vel_ref,
                     const Eigen::Vector3d &acc_ref, const Eigen::Quaterniond &quat_ref,
                     const Eigen::Vector3d &pos_fdb, const Eigen::Vector3d &vel_fdb);
};





#endif