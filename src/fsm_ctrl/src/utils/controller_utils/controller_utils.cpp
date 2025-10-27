/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-27 21
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

    nh.param("/single_offboard_fsm/ipopt_parameters/single_predict_horizon", nmpc_single_predict_horizon, 2.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/predict_steps", nmpc_predict_steps, 20.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/max_w_limit", nmpc_max_w_limit, 2.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/min_w_limit", nmpc_min_w_limit, -2.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/max_acc_z_limit", nmpc_max_acc_z_limit, 5.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/min_acc_z_limit", nmpc_min_acc_z_limit, -5.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/max_force_limit", nmpc_max_force_limit, 15.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/min_force_limit", nmpc_min_force_limit, 0.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/max_dforce_limit", nmpc_max_dforce_limit, 5.0);
    nh.param("/single_offboard_fsm/ipopt_parameters/min_dforce_limit", nmpc_min_dforce_limit, -5.0);

    float nmpc_Q_pos_x, nmpc_Q_pos_y, nmpc_Q_pos_z;
    float nmpc_Q_vel_x, nmpc_Q_vel_y, nmpc_Q_vel_z;
    float nmpc_Q_quat_x, nmpc_Q_quat_y, nmpc_Q_quat_z;
    float nmpc_R_w_x, nmpc_R_w_y, nmpc_R_w_z;
    float nmpc_R_acc_z;

    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_pos_x", nmpc_Q_pos_x, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_pos_y", nmpc_Q_pos_y, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_pos_z", nmpc_Q_pos_z, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_vel_x", nmpc_Q_vel_x, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_vel_y", nmpc_Q_vel_y, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_vel_z", nmpc_Q_vel_z, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_quat_x", nmpc_Q_quat_x, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_quat_y", nmpc_Q_quat_y, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/Q_quat_z", nmpc_Q_quat_z, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/R_w_x", nmpc_R_w_x, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/R_w_y", nmpc_R_w_y, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/R_w_z", nmpc_R_w_z, 1.0f);
    nh.param("/single_offboard_fsm/ipopt_parameters/w_and_totalF_params/R_acc_z", nmpc_R_acc_z, 1.0f);

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

void DFBCControllerInit(ros::NodeHandle& nh, DFBC_Controller& dfbc_controller)
{
    int ctrl_rate;
    double hover_thrust_percentage;
    double kp_pos_x, kp_pos_y, kp_pos_z;
    double ki_pos_x, ki_pos_y, ki_pos_z;
    double kd_vel_x, kd_vel_y, kd_vel_z;
    double kp_vel_x, kp_vel_y, kp_vel_z;

    nh.param("/single_offboard_fsm/controller_basic_params/hover_thrust", hover_thrust_percentage, 0.1);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_pos_x", kp_pos_x, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_pos_y", kp_pos_y, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_pos_z", kp_pos_z, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/ki_pos_x", ki_pos_x, 0.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/ki_pos_y", ki_pos_y, 0.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/ki_pos_z", ki_pos_z, 0.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kd_vel_x", kd_vel_x, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kd_vel_y", kd_vel_y, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kd_vel_z", kd_vel_z, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_vel_x", kp_vel_x, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_vel_y", kp_vel_y, 1.0);
    nh.param("/single_offboard_fsm/dfbc_parameters/kp_vel_z", kp_vel_z, 1.0);

    Eigen::Vector3d kp_pos, ki_pos, kd_vel, kp_vel;
    kp_pos << kp_pos_x, kp_pos_y, kp_pos_z;
    ki_pos << ki_pos_x, ki_pos_y, ki_pos_z;
    kd_vel << kd_vel_x, kd_vel_y, kd_vel_z;
    kp_vel << kp_vel_x, kp_vel_y, kp_vel_z;

    dfbc_controller.Init(const_params::RATE, hover_thrust_percentage, kp_pos, ki_pos, kd_vel, kp_vel);
}

void AcadosNmpcWandTotalFControllerInit(ros::NodeHandle& nh, AcadosSimpleController& acados_controller)
{
    acados_controller.acados_ocp_capsule = w_totalF_nmpc_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    // int N = const_params::W_TOTALF_PARAMS.NP;
    // allocate the array and fill it accordingly
    acados_controller.new_time_steps = NULL;
    acados_controller.status = w_totalF_nmpc_acados_create_with_discretization(acados_controller.acados_ocp_capsule, acados_controller.N, acados_controller.new_time_steps);

    if (acados_controller.status)
    {
        printf("w_totalF_nmpc_acados_create() returned status %d. Exiting.\n", acados_controller.status);
        exit(1);
    }

    acados_controller.nlp_config = w_totalF_nmpc_acados_get_nlp_config(acados_controller.acados_ocp_capsule);
    acados_controller.nlp_dims = w_totalF_nmpc_acados_get_nlp_dims(acados_controller.acados_ocp_capsule);
    acados_controller.nlp_in = w_totalF_nmpc_acados_get_nlp_in(acados_controller.acados_ocp_capsule);
    acados_controller.nlp_out = w_totalF_nmpc_acados_get_nlp_out(acados_controller.acados_ocp_capsule);
    acados_controller.nlp_solver = w_totalF_nmpc_acados_get_nlp_solver(acados_controller.acados_ocp_capsule);
    acados_controller.nlp_opts = w_totalF_nmpc_acados_get_nlp_opts(acados_controller.acados_ocp_capsule);

    for(int i = 0; i < const_params::W_TOTALF_PARAMS.NBX0; i++)
    {
        acados_controller.lbx0[i] = 0.0;
        acados_controller.ubx0[i] = 0.0;
        acados_controller.x_init[i] = 0.0;
    }
    acados_controller.lbx0[6] = 1.0;
    acados_controller.ubx0[6] = 1.0;
    acados_controller.x_init[6] = 1.0;

    acados_controller.u0[0] = 9.81;
    acados_controller.u0[1] = 0.0;
    acados_controller.u0[2] = 0.0;
    acados_controller.u0[3] = 0.0;

    acados_controller.params[0] = 1.0;
    acados_controller.params[1] = 0.0;
    acados_controller.params[2] = 0.0;
    acados_controller.params[3] = 0.0;

    ocp_nlp_constraints_model_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_in, 0, "lbx", acados_controller.lbx0);
    ocp_nlp_constraints_model_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_in, 0, "ubx", acados_controller.ubx0);

    acados_controller.NTIMINGS = 1;
    acados_controller.min_time = 1e12;

    // solve ocp in loop
    for (int ii = 0; ii < acados_controller.NTIMINGS; ii++)
    {
        // initialize solution
        double* yref_0 = static_cast<double*>(calloc(const_params::W_TOTALF_PARAMS.NY0, sizeof(double)));
        // change only the non-zero elements:
        yref_0[2] = 1.0;
        yref_0[9] = 9.8015;
        ocp_nlp_cost_model_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_in, 0, "yref", yref_0);

        double* yref = static_cast<double*>(calloc(const_params::W_TOTALF_PARAMS.NY, sizeof(double)));
        // change only the non-zero elements:
        yref[2] = 1.0;
        yref[9] = 9.8015;

        for (int i = 1; i < acados_controller.N; i++)
        {
            ocp_nlp_cost_model_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_in, i, "yref", yref);
        }
        free(yref);

        double* yref_e = static_cast<double*>(calloc(const_params::W_TOTALF_PARAMS.NYN, sizeof(double)));
        // change only the non-zero elements:
        yref_e[2] = 1.0;
        ocp_nlp_cost_model_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_in, acados_controller.N, "yref", yref_e);
        free(yref_e);

        for (int i = 0; i < acados_controller.N; i++)
        {
            ocp_nlp_out_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, i, "x", acados_controller.x_init);
            ocp_nlp_out_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, i, "u", acados_controller.u0);
            acados_controller.status = w_totalF_nmpc_acados_update_params(acados_controller.acados_ocp_capsule, i, acados_controller.params, const_params::W_TOTALF_PARAMS.NP);
        }
        ocp_nlp_out_set(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, acados_controller.N, "x", acados_controller.x_init);
        acados_controller.status = w_totalF_nmpc_acados_update_params(acados_controller.acados_ocp_capsule, acados_controller.N, acados_controller.params, 4);
        acados_controller.status = w_totalF_nmpc_acados_solve(acados_controller.acados_ocp_capsule);
        ocp_nlp_get(acados_controller.nlp_solver, "time_tot", &acados_controller.elapsed_time);
        acados_controller.min_time = MIN(acados_controller.elapsed_time, acados_controller.min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= acados_controller.nlp_dims->N; ii++)
        ocp_nlp_out_get(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, ii, "x", &acados_controller.xtraj[ii*const_params::W_TOTALF_PARAMS.NX]);
    for (int ii = 0; ii < acados_controller.nlp_dims->N; ii++)
        ocp_nlp_out_get(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, ii, "u", &acados_controller.utraj[ii*const_params::W_TOTALF_PARAMS.NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( const_params::W_TOTALF_PARAMS.NX, acados_controller.N+1, acados_controller.xtraj, const_params::W_TOTALF_PARAMS.NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( const_params::W_TOTALF_PARAMS.NU, acados_controller.N, acados_controller.utraj, const_params::W_TOTALF_PARAMS.NU);
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", acados_controller.NTIMINGS);

    if (acados_controller.status == ACADOS_SUCCESS)
    {
        printf("w_totalF_nmpc_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("w_totalF_nmpc_acados_solve() failed with status %d.\n", acados_controller.status);
    }

    // get solution
    ocp_nlp_out_get(acados_controller.nlp_config, acados_controller.nlp_dims, acados_controller.nlp_out, 0, "kkt_norm_inf", &acados_controller.kkt_norm_inf);
    ocp_nlp_get(acados_controller.nlp_solver, "sqp_iter", &acados_controller.sqp_iter);

    w_totalF_nmpc_acados_print_stats(acados_controller.acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           acados_controller.sqp_iter, acados_controller.NTIMINGS, acados_controller.min_time*1000, acados_controller.kkt_norm_inf);
}

} // namespace fsm_ut