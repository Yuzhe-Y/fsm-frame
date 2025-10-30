/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-30 11
 * @FilePath: /fsm_ctrl/src/callbacks/timer_callback.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/callbacks/timer_callback.hpp"

namespace fsm_cb
{
    void IpoptNmpcWandTotalFTimerCallback(const ros::TimerEvent& event)
    {
        if(fsm.controller_work_enable)
        {
            fsm_ut::SetControllerFdb(fsm.controller);
            std::vector<double> current_states;
            std::vector<double> desired_states;

            current_states.push_back(fsm.controller.position_fdb.x());
            current_states.push_back(fsm.controller.position_fdb.y());
            current_states.push_back(fsm.controller.position_fdb.z());
            current_states.push_back(fsm.controller.velocity_fdb.x());
            current_states.push_back(fsm.controller.velocity_fdb.y());
            current_states.push_back(fsm.controller.velocity_fdb.z());
            current_states.push_back(fsm.controller.attitude_fdb.w());
            current_states.push_back(fsm.controller.attitude_fdb.x());
            current_states.push_back(fsm.controller.attitude_fdb.y());
            current_states.push_back(fsm.controller.attitude_fdb.z());

            // for(int i = 0; i < fsm.nmpc_controller_w_and_totalF.getNLPPredictStep() + 1; i++)
            // {
            //     desired_states.push_back(fsm.controller.position_ref[i].x());
            //     desired_states.push_back(fsm.controller.position_ref[i].y());
            //     desired_states.push_back(fsm.controller.position_ref[i].z());
            //     desired_states.push_back(fsm.controller.velocity_ref[i].x());
            //     desired_states.push_back(fsm.controller.velocity_ref[i].y());
            //     desired_states.push_back(fsm.controller.velocity_ref[i].z());
            //     desired_states.push_back(fsm.controller.attitude_ref[i].w());
            //     desired_states.push_back(fsm.controller.attitude_ref[i].x());
            //     desired_states.push_back(fsm.controller.attitude_ref[i].y());
            //     desired_states.push_back(fsm.controller.attitude_ref[i].z());
            // }

            // for(int i = 0; i < fsm.nmpc_controller_w_and_totalF.getNLPPredictStep(); i++)
            // {
            //     desired_states.push_back(fsm.controller.angular_velocity_ref[i].x());
            //     desired_states.push_back(fsm.controller.angular_velocity_ref[i].y());
            //     desired_states.push_back(fsm.controller.angular_velocity_ref[i].z());
            //     desired_states.push_back(fsm.controller.acc_z_ref[i]);
            // }

            // fsm.nmpc_controller_w_and_totalF.optimal_solution(current_states, desired_states);

            // double acc_z_command = fsm.nmpc_controller_w_and_totalF.getTotalFCommand();
            // Eigen::Vector3d w_command = fsm.nmpc_controller_w_and_totalF.getwCommand();

            // mavros_msgs::AttitudeTarget att_tgt = fsm_ut::SetTargetRateAndTotalThrustCmd(w_command.x(), w_command.y(), w_command.z(), acc_z_command);
            // setpoint_raw_att_pub.publish(att_tgt);
            fsm_ctrl::nmpc_simple_model_msgs nmpc_state_msgs;
            nmpc_state_msgs.header.stamp = ros::Time::now();
            nmpc_state_msgs.header.frame_id = "nmpc_state";
            nmpc_state_msgs.pos.position.x = fsm.controller.position_fdb.x();
            nmpc_state_msgs.pos.position.y = fsm.controller.position_fdb.y();
            nmpc_state_msgs.pos.position.z = fsm.controller.position_fdb.z();

            // nmpc_state_pub.publish(nmpc_state_msgs);
        }
        else
        {
            if(std::floor(fsm.now_cmd_time) - std::floor(fsm.last_cmd_time) > 0)
            {
                ROS_WARN("Controller not work!");
            }
        }
    }

    void DFBCTimerCallback(const ros::TimerEvent& event)
    {
        if(fsm.controller_work_enable)
        {
            fsm_ut::SetControllerFdb(fsm.controller);
            fsm.dfbc_controller.AttCtrl(fsm.controller.position_ref[0], fsm.controller.velocity_ref[0],
                                        fsm.controller.acceleration_ref[0], fsm.controller.attitude_ref[0],
                                        fsm.controller.position_fdb, fsm.controller.velocity_fdb);

            // mavros_msgs::AttitudeTarget att_tgt = fsm_ut::SetTargetAttAndTotalThrustCmd(fsm.dfbc_controller.quat_command.x(), fsm.dfbc_controller.quat_command.y(),
            //                                                                             fsm.dfbc_controller.quat_command.z(), fsm.dfbc_controller.quat_command.w(),
            //                                                                             fsm.dfbc_controller.thrust_percentage);
            // setpoint_raw_att_pub.publish(att_tgt);
            fsm_ctrl::dfbc_msgs dfbc_state_msgs;
            dfbc_state_msgs.header.stamp = ros::Time::now();
            dfbc_state_msgs.header.frame_id = "dfbc_state";
            dfbc_state_msgs.pos_fdb.position.x = fsm.controller.position_fdb.x();
            dfbc_state_msgs.pos_fdb.position.y = fsm.controller.position_fdb.y();
            dfbc_state_msgs.pos_fdb.position.z = fsm.controller.position_fdb.z();

            // dfbc_state_pub.publish(dfbc_state_msgs);
        }
        else
        {
            if(std::floor(fsm.now_cmd_time) - std::floor(fsm.last_cmd_time) > 0)
            {
                ROS_WARN("Controller not work!");
            }
        }
    }

    void AcadosNmpcSimpleModelTimerCallback(const ros::TimerEvent& event)
    {
        if(fsm.controller_work_enable)
        {
            fsm_ut::SetControllerFdb(fsm.controller);

            fsm.acados_simple_controller.nlp_config = w_totalF_nmpc_acados_get_nlp_config(fsm.acados_simple_controller.acados_ocp_capsule);
            fsm.acados_simple_controller.nlp_dims = w_totalF_nmpc_acados_get_nlp_dims(fsm.acados_simple_controller.acados_ocp_capsule);
            fsm.acados_simple_controller.nlp_in = w_totalF_nmpc_acados_get_nlp_in(fsm.acados_simple_controller.acados_ocp_capsule);
            fsm.acados_simple_controller.nlp_out = w_totalF_nmpc_acados_get_nlp_out(fsm.acados_simple_controller.acados_ocp_capsule);
            fsm.acados_simple_controller.nlp_solver = w_totalF_nmpc_acados_get_nlp_solver(fsm.acados_simple_controller.acados_ocp_capsule);
            fsm.acados_simple_controller.nlp_opts = w_totalF_nmpc_acados_get_nlp_opts(fsm.acados_simple_controller.acados_ocp_capsule);

            fsm.acados_simple_controller.x_init[0] = fsm.controller.position_fdb.x();
            fsm.acados_simple_controller.x_init[1] = fsm.controller.position_fdb.y();
            fsm.acados_simple_controller.x_init[2] = fsm.controller.position_fdb.z();
            fsm.acados_simple_controller.x_init[3] = fsm.controller.velocity_fdb.x();
            fsm.acados_simple_controller.x_init[4] = fsm.controller.velocity_fdb.y();
            fsm.acados_simple_controller.x_init[5] = fsm.controller.velocity_fdb.z();
            fsm.acados_simple_controller.x_init[6] = fsm.controller.attitude_fdb.w();
            fsm.acados_simple_controller.x_init[7] = fsm.controller.attitude_fdb.x();
            fsm.acados_simple_controller.x_init[8] = fsm.controller.attitude_fdb.y();
            fsm.acados_simple_controller.x_init[9] = fsm.controller.attitude_fdb.z();

            fsm.acados_simple_controller.u0[0] = 9.8015;
            fsm.acados_simple_controller.u0[1] = 0.0;
            fsm.acados_simple_controller.u0[2] = 0.0;
            fsm.acados_simple_controller.u0[3] = 0.0;

            ocp_nlp_constraints_model_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_in, 0, "lbx", fsm.acados_simple_controller.x_init);
            ocp_nlp_constraints_model_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_in, 0, "ubx", fsm.acados_simple_controller.x_init);

            for (int ii = 0; ii < fsm.acados_simple_controller.NTIMINGS; ii++)
            {
                for(int i = 0; i < fsm.acados_simple_controller.N; i++)
                {
                    double* yref = static_cast<double*>(calloc(const_params::W_TOTALF_PARAMS.NY, sizeof(double)));
                    yref[0] = fsm.controller.position_ref[i].x();
                    yref[1] = fsm.controller.position_ref[i].y();
                    yref[2] = fsm.controller.position_ref[i].z();
                    yref[3] = fsm.controller.velocity_ref[i].x();
                    yref[4] = fsm.controller.velocity_ref[i].y();
                    yref[5] = fsm.controller.velocity_ref[i].z();
                    yref[9] = const_params::GRAVITY;
                    fsm.acados_simple_controller.params[0] = fsm.controller.attitude_ref[i].w();
                    fsm.acados_simple_controller.params[1] = fsm.controller.attitude_ref[i].x();
                    fsm.acados_simple_controller.params[2] = fsm.controller.attitude_ref[i].y();
                    fsm.acados_simple_controller.params[3] = fsm.controller.attitude_ref[i].z();
                    ocp_nlp_cost_model_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_in, i, "yref", yref);
                    fsm.acados_simple_controller.status = w_totalF_nmpc_acados_update_params(fsm.acados_simple_controller.acados_ocp_capsule, i, fsm.acados_simple_controller.params, const_params::W_TOTALF_PARAMS.NP);
                    free(yref);
                }
            
                double* yref_e = static_cast<double*>(calloc(const_params::W_TOTALF_PARAMS.NYN, sizeof(double)));
                yref_e[0] = fsm.controller.position_ref[fsm.acados_simple_controller.N].x();
                yref_e[1] = fsm.controller.position_ref[fsm.acados_simple_controller.N].y();
                yref_e[2] = fsm.controller.position_ref[fsm.acados_simple_controller.N].z();
                yref_e[3] = fsm.controller.velocity_ref[fsm.acados_simple_controller.N].x();
                yref_e[4] = fsm.controller.velocity_ref[fsm.acados_simple_controller.N].y();
                yref_e[5] = fsm.controller.velocity_ref[fsm.acados_simple_controller.N].z();
                fsm.acados_simple_controller.params[0] = fsm.controller.attitude_ref[fsm.acados_simple_controller.N].w();
                fsm.acados_simple_controller.params[1] = fsm.controller.attitude_ref[fsm.acados_simple_controller.N].x();
                fsm.acados_simple_controller.params[2] = fsm.controller.attitude_ref[fsm.acados_simple_controller.N].y();
                fsm.acados_simple_controller.params[3] = fsm.controller.attitude_ref[fsm.acados_simple_controller.N].z();
                ocp_nlp_cost_model_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_in, fsm.acados_simple_controller.N, "yref", yref_e);
                fsm.acados_simple_controller.status = w_totalF_nmpc_acados_update_params(fsm.acados_simple_controller.acados_ocp_capsule, fsm.acados_simple_controller.N, fsm.acados_simple_controller.params, const_params::W_TOTALF_PARAMS.NP);
                free(yref_e);
            
                for (int i = 0; i < fsm.acados_simple_controller.N; i++)
                {
                    ocp_nlp_out_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, i, "x", fsm.acados_simple_controller.x_init);
                    ocp_nlp_out_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, i, "u", fsm.acados_simple_controller.u0);
                }
                ocp_nlp_out_set(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, fsm.acados_simple_controller.N, "x", fsm.acados_simple_controller.x_init);

                fsm.acados_simple_controller.status = w_totalF_nmpc_acados_solve(fsm.acados_simple_controller.acados_ocp_capsule);
                ocp_nlp_get(fsm.acados_simple_controller.nlp_solver, "time_tot", &fsm.acados_simple_controller.elapsed_time);
                fsm.acados_simple_controller.min_time = MIN(fsm.acados_simple_controller.elapsed_time, fsm.acados_simple_controller.min_time);
            }
        
            /* print solution and statistics */
            for (int ii = 0; ii <= fsm.acados_simple_controller.nlp_dims->N; ii++)
                ocp_nlp_out_get(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, ii, "x", &fsm.acados_simple_controller.xtraj[ii*const_params::W_TOTALF_PARAMS.NX]);
            for (int ii = 0; ii < fsm.acados_simple_controller.nlp_dims->N; ii++)
                ocp_nlp_out_get(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, ii, "u", &fsm.acados_simple_controller.utraj[ii*const_params::W_TOTALF_PARAMS.NU]);

            // get solution
            ocp_nlp_out_get(fsm.acados_simple_controller.nlp_config, fsm.acados_simple_controller.nlp_dims, fsm.acados_simple_controller.nlp_out, 0, "kkt_norm_inf", &fsm.acados_simple_controller.kkt_norm_inf);
            ocp_nlp_get(fsm.acados_simple_controller.nlp_solver, "sqp_iter", &fsm.acados_simple_controller.sqp_iter);

            double thrust_percentage_command = fsm.acados_simple_controller.thr_est.LinearThrEst(fsm.acados_simple_controller.utraj[0]);
            Eigen::Vector3d w_command;
            w_command.x() = fsm.acados_simple_controller.utraj[1];
            w_command.y() = fsm.acados_simple_controller.utraj[2];
            w_command.z() = fsm.acados_simple_controller.utraj[3];

            mavros_msgs::AttitudeTarget att_tgt = fsm_ut::SetTargetRateAndTotalThrustCmd(w_command.x(), w_command.y(), w_command.z(), thrust_percentage_command);
            fsm.setpoint_raw_att_pub.publish(att_tgt);

            fsm_ctrl::nmpc_simple_model_msgs nmpc_state_msgs;
            nmpc_state_msgs.header.stamp = ros::Time::now();
            nmpc_state_msgs.header.frame_id = "nmpc_state";
            nmpc_state_msgs.body_rate_cmd.x = w_command.x();
            nmpc_state_msgs.body_rate_cmd.y = w_command.y();
            nmpc_state_msgs.body_rate_cmd.z = w_command.z();
            nmpc_state_msgs.thrust_cmd = thrust_percentage_command;

            fsm.nmpc_state_pub.publish(nmpc_state_msgs);
        }
        else
        {
            if(std::floor(fsm.now_cmd_time) - std::floor(fsm.last_cmd_time) > 0)
            {
                ROS_WARN("Controller not work!");
            }
        }
    }
}