/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-25 16
 * @FilePath: /fsm_ctrl/src/utils/controller_utils/controller_utils.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/utils/controller_utils/controller_utils.hpp"

namespace fsm_ut
{

void SetControllerFdb(Controller& controller)
{
    // 设置位置反馈
    controller.position_fdb = fsm_cb::mavros_fcu_pos;
    // 设置速度反馈
    controller.velocity_fdb = fsm_cb::mavros_fcu_vel;
    // 设置姿态反馈
    controller.attitude_fdb = fsm_cb::mavros_fcu_quat;
    // 设置角速度反馈
    controller.angular_velocity_fdb = fsm_cb::mavros_imu_rate;
}

void IpoptNmpcWandTotalFControllerInit(ros::NodeHandle& nh, NMPC_Ctrller_simple& nmpc_controller)
{
    double nmpc_single_predict_horizon, nmpc_predict_steps;
    double nmpc_max_w_limit, nmpc_min_w_limit;
    double nmpc_max_acc_z_limit, nmpc_min_acc_z_limit;
    double nmpc_max_force_limit, nmpc_min_force_limit;
    double nmpc_max_dforce_limit, nmpc_min_dforce_limit;

    nh.param("/ipopt_parameters/single_predict_horizon", nmpc_single_predict_horizon, 2.0);
    nh.param("/ipopt_parameters/predict_steps", nmpc_predict_steps, 20.0);
    nh.param("/ipopt_parameters/max_w_limit", nmpc_max_w_limit, 2.0);
    nh.param("/ipopt_parameters/min_w_limit", nmpc_min_w_limit, -2.0);
    nh.param("/ipopt_parameters/max_acc_z_limit", nmpc_max_acc_z_limit, 5.0);
    nh.param("/ipopt_parameters/min_acc_z_limit", nmpc_min_acc_z_limit, -5.0);
    nh.param("/ipopt_parameters/max_force_limit", nmpc_max_force_limit, 15.0);
    nh.param("/ipopt_parameters/min_force_limit", nmpc_min_force_limit, 0.0);
    nh.param("/ipopt_parameters/max_dforce_limit", nmpc_max_dforce_limit, 5.0);
    nh.param("/ipopt_parameters/min_dforce_limit", nmpc_min_dforce_limit, -5.0);

    float nmpc_Q_pos_x, nmpc_Q_pos_y, nmpc_Q_pos_z;
    float nmpc_Q_vel_x, nmpc_Q_vel_y, nmpc_Q_vel_z;
    float nmpc_Q_quat_x, nmpc_Q_quat_y, nmpc_Q_quat_z;
    float nmpc_R_w_x, nmpc_R_w_y, nmpc_R_w_z;
    float nmpc_R_acc_z;

    nh.param("/ipopt_parameters/w_and_totalF_params/Q_pos_x", nmpc_Q_pos_x, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_pos_y", nmpc_Q_pos_y, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_pos_z", nmpc_Q_pos_z, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_vel_x", nmpc_Q_vel_x, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_vel_y", nmpc_Q_vel_y, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_vel_z", nmpc_Q_vel_z, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_quat_x", nmpc_Q_quat_x, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_quat_y", nmpc_Q_quat_y, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/Q_quat_z", nmpc_Q_quat_z, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/R_w_x", nmpc_R_w_x, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/R_w_y", nmpc_R_w_y, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/R_w_z", nmpc_R_w_z, 1.0f);
    nh.param("/ipopt_parameters/w_and_totalF_params/R_acc_z", nmpc_R_acc_z, 1.0f);

    std::array<double, 2> _w_limit = {nmpc_min_w_limit, nmpc_max_w_limit};
    std::array<double, 2> _acc_z_limit = {nmpc_min_acc_z_limit, nmpc_max_acc_z_limit};
    Eigen::Matrix<float, 3, 1> nmpc_costQpos = {nmpc_Q_pos_x, nmpc_Q_pos_y, nmpc_Q_pos_z};
    Eigen::Matrix<float, 3, 1> nmpc_costQvel = {nmpc_Q_vel_x, nmpc_Q_vel_y, nmpc_Q_vel_z};
    Eigen::Matrix<float, 3, 1> nmpc_costRw = {nmpc_R_w_x, nmpc_R_w_y, nmpc_R_w_z};
    Eigen::Matrix<float, 3, 1> nmpc_costQquat = {nmpc_Q_quat_x, nmpc_Q_quat_y, nmpc_Q_quat_z};

    NMPC_Ctrller_simple tmp_controller(const_params::INTERV, 
                                       _acc_z_limit, 
                                       _w_limit,
                                       nmpc_predict_steps, 
                                       nmpc_single_predict_horizon, 
                                       10, 
                                       4,
                                       nmpc_costQpos,
                                       nmpc_costQvel,
                                       nmpc_costRw,
                                       nmpc_costQquat,
                                       nmpc_R_acc_z); //构造函数

    nmpc_controller = tmp_controller;
}

} // namespace fsm_ut