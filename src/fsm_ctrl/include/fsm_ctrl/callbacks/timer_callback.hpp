/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-29 15
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/callbacks/timer_callback.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __TIMER_CALLBACK_H__
#define __TIMER_CALLBACK_H__

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_w_totalF_nmpc.h"

#include "blasfeo_d_aux_ext_dep.h"

#include "fsm_ctrl/utils/controller_utils/controller_utils.hpp"
#include "fsm_ctrl/utils/communication_utils/px4_command.hpp"
#include "fsm_ctrl/controllers/ipopt_controller/nmpc_controller.hpp"
#include "fsm_ctrl/nodes/single_offboard_fsm.hpp"

#include "fsm_ctrl/nmpc_simple_model_msgs.h"


#include "const_params.h"

namespace fsm_cb
{
    void AcadosNmpcSimpleModelTimerCallback(const ros::TimerEvent& event);
    void IpoptNmpcWandTotalFTimerCallback(const ros::TimerEvent& event);
}

#endif // __TIMER_CALLBACK_H__