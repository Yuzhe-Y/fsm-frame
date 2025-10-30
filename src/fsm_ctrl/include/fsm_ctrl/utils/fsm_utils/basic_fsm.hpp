/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-29 18
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/utils/fsm_utils/basic_fsm.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _BASIC_FSM_HPP_
#define _BASIC_FSM_HPP_

// Basic include 0
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream> 
#include <map>
#include <stdexcept>
#include <vector>
#include <deque>
#include <functional>

// Basic include 1
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

// MAVROS and message include
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include <tf/tf.h>

// Eigen and project include
#include <eigen3/Eigen/Dense>
#include "fsm_ctrl/utils/math_utils/math_utils.hpp"
#include "fsm_ctrl/utils/controller_utils/controller_utils.hpp"
#include "fsm_ctrl/callbacks/mavros_callback.hpp"
#include "fsm_ctrl/callbacks/external_odom_callback.hpp"
#include "fsm_ctrl/callbacks/timer_callback.hpp"
#include "fsm_ctrl/controllers/ipopt_controller/nmpc_controller.hpp"
#include "fsm_ctrl/controllers/dfbc_controller/dfbc_controller.hpp"
#include "fsm_ctrl/utils/communication_utils/px4_command.hpp"

#include <fsm_ctrl/nmpc_simple_model_msgs.h>
#include <fsm_ctrl/dfbc_msgs.h>

#include "const_params.h"

// 临时增加控制头文件
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_w_totalF_nmpc.h"

#include "blasfeo_d_aux_ext_dep.h"

#include "const_params.h"

namespace fsm_ut
{

class Basic_FSM
{
    public:
        /*--------- parameter ---------*/
        double first_takeoff_height;
        double hover_thrust_percentage;
        bool controller_work_enable;
        bool real_environment;

        /*--------- user UDP ---------*/
        bool is_udp_enable;
        int cmd;
        int last_cmd;

        /*--------- mode ---------*/
        std::string mavros_mode;
        bool arm_mode;
        bool is_ekf_converge;
        bool is_safety_trigger;
        mavros_msgs::SetMode offboard_mode, land_mode;
        mavros_msgs::CommandBool arm_cmd, disarm_cmd;

        /*--------- control ---------*/
        fsm_ut::Controller controller;
        const_params::Nmpc_Params nmpc_params;

        NMPC_Ctrller_simple nmpc_controller_w_and_totalF;
        DFBC_Controller dfbc_controller;
        fsm_ut::AcadosSimpleController acados_simple_controller;

        geometry_msgs::PoseStamped mavros_pos_cmd;
        geometry_msgs::TwistStamped mavros_vel_cmd;  
        mavros_msgs::PositionTarget mavros_pos_raw_cmd;
        mavros_msgs::AttitudeTarget mavros_att_raw_cmd;

        /*--------- ROS publisher ---------*/   
        ros::Publisher setpoint_pos_pub;
        ros::Publisher setpoint_vel_pub;
        ros::Publisher setpoint_raw_local_pub;
        ros::Publisher setpoint_raw_att_pub;
        ros::Publisher nmpc_state_pub;

        /*--------- ROS client ---------*/
        ros::ServiceClient arming_cmd_client;
        ros::ServiceClient set_mode_client;

        /*--------- ROS loop ---------*/
        double last_cmd_time;
        double now_cmd_time;
        double last_request;           //主程序开始时间
        ros::Time mavros_state_monitor_time;

        /*--------- landing variable ---------*/
        Eigen::Vector3d landing_start_pos;
        int landing_step_count;
        bool is_landing_in_progress;

        /*--------- Timer ---------*/
        ros::Timer controller_timer;

        /*--------- constructor ---------*/
        Basic_FSM();
        ~Basic_FSM() = default;

        /*--------- main ---------*/
        void UDPListen(const uint16_t cport);
        void Basic_Init(ros::NodeHandle &nh, ros::Rate rate);
        void Basic_Task();
        virtual void FLAG_Task() = 0;
};

class FLAG_FSM: public Basic_FSM
{
    public:
        FLAG_FSM();
        ~FLAG_FSM() = default;

        void FLAG_Task() override;
};

}

extern fsm_ut::FLAG_FSM fsm;


#endif