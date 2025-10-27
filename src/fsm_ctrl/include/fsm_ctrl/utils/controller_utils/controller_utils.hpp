/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-27 10
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

#include <stdio.h>
#include <stdlib.h>

#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_w_totalF_nmpc.h"

#include "blasfeo_d_aux_ext_dep.h"

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

    struct AcadosController
    {
        w_totalF_nmpc_solver_capsule *acados_ocp_capsule;
        const int N = const_params::W_TOTALF_PARAMS.NP; //预测步数
        double* new_time_steps;
        int status;

        ocp_nlp_config *nlp_config;
        ocp_nlp_dims *nlp_dims;
        ocp_nlp_in *nlp_in;
        ocp_nlp_out *nlp_out;
        ocp_nlp_solver *nlp_solver;
        void *nlp_opts;

        double lbx0[const_params::W_TOTALF_PARAMS.NBX0];
        double ubx0[const_params::W_TOTALF_PARAMS.NBX0];
        double x_init[const_params::W_TOTALF_PARAMS.NX];
        double u0[const_params::W_TOTALF_PARAMS.NU];

        int NTIMINGS;
        double min_time;
        double kkt_norm_inf;
        double elapsed_time;
        int sqp_iter;

        // double xtraj[const_params::W_TOTALF_PARAMS.NX * (N+1)];
        // double utraj[const_params::W_TOTALF_PARAMS.NU * N];
    };

    void SetControllerFdb(Controller& controller);

    void IpoptNmpcWandTotalFControllerInit(ros::NodeHandle& nh, NMPC_Ctrller_simple& nmpc_controller);
    void DFBCControllerInit(ros::NodeHandle& nh, DFBC_Controller& dfbc_controller);
    void AcadosNmpcWandTotalFControllerInit(ros::NodeHandle& nh);

} // namespace fsm_ut
#endif