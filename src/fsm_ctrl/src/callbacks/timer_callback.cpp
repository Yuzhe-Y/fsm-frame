/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-29 15
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
        fsm_ut::SetControllerFdb(controller);
        std::vector<double> current_states;
        std::vector<double> desired_states;

        current_states.push_back(controller.position_fdb.x());
        current_states.push_back(controller.position_fdb.y());
        current_states.push_back(controller.position_fdb.z());
        current_states.push_back(controller.velocity_fdb.x());
        current_states.push_back(controller.velocity_fdb.y());
        current_states.push_back(controller.velocity_fdb.z());
        current_states.push_back(controller.attitude_fdb.w());
        current_states.push_back(controller.attitude_fdb.x());
        current_states.push_back(controller.attitude_fdb.y());
        current_states.push_back(controller.attitude_fdb.z());

        // for(int i = 0; i < nmpc_controller_w_and_totalF.getNLPPredictStep() + 1; i++)
        // {
        //     desired_states.push_back(controller.position_ref[i].x());
        //     desired_states.push_back(controller.position_ref[i].y());
        //     desired_states.push_back(controller.position_ref[i].z());
        //     desired_states.push_back(controller.velocity_ref[i].x());
        //     desired_states.push_back(controller.velocity_ref[i].y());
        //     desired_states.push_back(controller.velocity_ref[i].z());
        //     desired_states.push_back(controller.attitude_ref[i].w());
        //     desired_states.push_back(controller.attitude_ref[i].x());
        //     desired_states.push_back(controller.attitude_ref[i].y());
        //     desired_states.push_back(controller.attitude_ref[i].z());
        // }

        // for(int i = 0; i < nmpc_controller_w_and_totalF.getNLPPredictStep(); i++)
        // {
        //     desired_states.push_back(controller.angular_velocity_ref[i].x());
        //     desired_states.push_back(controller.angular_velocity_ref[i].y());
        //     desired_states.push_back(controller.angular_velocity_ref[i].z());
        //     desired_states.push_back(controller.acc_z_ref[i]);
        // }

        // nmpc_controller_w_and_totalF.optimal_solution(current_states, desired_states);

        // double acc_z_command = nmpc_controller_w_and_totalF.getTotalFCommand();
        // Eigen::Vector3d w_command = nmpc_controller_w_and_totalF.getwCommand();

        // mavros_msgs::AttitudeTarget att_tgt = fsm_ut::SetTargetRateAndTotalThrustCmd(w_command.x(), w_command.y(), w_command.z(), acc_z_command);
        // setpoint_raw_att_pub.publish(att_tgt);
        fsm_ctrl::nmpc_simple_model_msgs nmpc_state_msgs;
        nmpc_state_msgs.header.stamp = ros::Time::now();
        nmpc_state_msgs.header.frame_id = "nmpc_state";
        nmpc_state_msgs.pos.position.x = controller.position_fdb.x();
        nmpc_state_msgs.pos.position.y = controller.position_fdb.y();
        nmpc_state_msgs.pos.position.z = controller.position_fdb.z();

        nmpc_state_pub.publish(nmpc_state_msgs);
    }
}