/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-23 14
 * @FilePath: /fsm_ctrl/config/const_params.h
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#pragma once

// 引入 acados 自动生成的参数定义
#include "acados_solver_w_totalF_nmpc.h"

// 自己的命名空间，避免全局污染
namespace const_params
{
    struct Nmpc_Params {
        int NX;
        int NP;
        int NU;
        int NBX0;
        int NP_GLOBAL;
    };

    // 两套不同控制器的参数
    constexpr Nmpc_Params W_TOTALF_PARAMS {
        W_TOTALF_NMPC_NX,
        W_TOTALF_NMPC_NP,
        W_TOTALF_NMPC_NU,
        W_TOTALF_NMPC_NBX0,
        W_TOTALF_NMPC_NP_GLOBAL
    };

    constexpr double RATE = 50.0;
    constexpr double INTERV = 1.0/RATE;

    constexpr double GRAVITY = 9.8015; // 重力加速度
    constexpr double COS45DEGREE = 0.7071067811865475; // cos(45°)

    constexpr double DRONE_MASS = 0.9393; // 无人机质量0.9
    constexpr double DRONE_ARM_L = 0.125; // 无人机机臂长度0.16
    constexpr double DRONE_CM = 5.632505e-10; // 电机反扭力系数4.888486266072161e-06
    constexpr double DRONE_CT = 1.4545236059e-8; // 电机推力系数0.000367717
    constexpr double DRONE_INERTIA_XX = 0.003281208642; // 无人机旋转惯量Ixx
    constexpr double DRONE_INERTIA_YY = 0.00312318016824; // 无人机旋转惯量Iyy
    constexpr double DRONE_INERTIA_ZZ = 0.00684939202037; // 无人机旋转惯量Izz
}
