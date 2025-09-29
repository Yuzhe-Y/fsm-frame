/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-29 15
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/nodes/single_offboard_fsm.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _SINGLE_OFFBOARD_FSM_HPP_
#define _SINGLE_OFFBOARD_FSM_HPP_

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
#include "fsm_ctrl/utils/communication_utils/px4_command.hpp"

#include <fsm_ctrl/nmpc_simple_model_msgs.h>

#include "const_params.h"

extern ros::Publisher setpoint_pos_pub;
extern ros::Publisher setpoint_vel_pub;
extern ros::Publisher setpoint_raw_local_pub;
extern ros::Publisher setpoint_raw_att_pub;
extern ros::Publisher nmpc_state_pub;

extern fsm_ut::Controller controller;
extern NMPC_Ctrller_simple nmpc_controller_w_and_totalF;

#endif