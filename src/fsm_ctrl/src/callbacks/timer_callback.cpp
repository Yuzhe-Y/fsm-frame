/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-24 11
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
}