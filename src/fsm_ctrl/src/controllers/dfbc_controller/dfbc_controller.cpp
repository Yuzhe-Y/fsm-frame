/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-04 15
 * @FilePath: /fsm_ctrl/src/controllers/dfbc_controller/dfbc_controller.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/controllers/dfbc_controller/dfbc_controller.hpp"

/**
 * @fn     DFBC
 * @brief  DFBC constructer
 * @param  NULL 
 * @return NULL
 */
DFBC_Controller::DFBC_Controller()
{
    Kp.setZero();
    Ki.setZero();
    Kd.setZero();
    Kv.setZero();
}


/**
 * @fn     Init
 * @brief  initialize differential flatness based controller
 * @param  _ctrl_rate: control rate
 * @param  _hover_thr: hover thrust
 * @param  kp, ki, kd, kv: PID gain
 * @return NULL 
 */
void DFBC_Controller::Init(double _ctrl_rate, double _hover_thr, const Eigen::Vector3d &kp, Eigen::Vector3d &ki, Eigen::Vector3d &kd, Eigen::Vector3d &kv)
{
    Kp.diagonal() = kp;
    Ki.diagonal() = ki;
    Kd.diagonal() = kd;
    Kv.diagonal() = kv;
    thr_est.Init_ThrEst(_ctrl_rate, _hover_thr);
}


/**
 * @fn     AttCtrl
 * @brief  differential flatness based attitude control
 * @param  ref: reference control point
 * @param  pos_fdb, vel_fdb: position & velocity feedback
 * @return att_cmd: attitude command
 */
void DFBC_Controller::AttCtrl(const Eigen::Vector3d &pos_ref, const Eigen::Vector3d &vel_ref,
                                                     const Eigen::Vector3d &acc_ref, const Eigen::Quaterniond &quat_ref,
                                                     const Eigen::Vector3d &pos_fdb, const Eigen::Vector3d &vel_fdb)
{
    /* error */
    static Eigen::Vector3d pos_err_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_err = pos_ref - pos_fdb;
    Eigen::Vector3d vel_err = vel_ref - vel_fdb;
    pos_err_i += pos_err;
    pos_err_i = Eigen::Vector3d(fsm_ut::Clamp(pos_err_i(0), 3.0), fsm_ut::Clamp(pos_err_i(1), 3.0), fsm_ut::Clamp(pos_err_i(2), 3.0));

    /* position loop: PID */
    /* velocity loop: FF-P */
    Eigen::Vector3d vel_des = Kp*pos_err + Ki*pos_err_i + Kd*vel_err;
    Eigen::Vector3d acc_des = acc_ref + Kv*(vel_des - vel_fdb) + Eigen::Vector3d(0.0, 0.0, const_params::GRAVITY);
    
    /* differential flatness */
    double thrust_percentage = thr_est.LinearThrEst(acc_des.norm());
    Eigen::Vector3d euler_ref = fsm_ut::QuatToEuler(quat_ref);
    Eigen::Matrix3d rot_des = fsm_ut::DiffFlat(acc_des, euler_ref.z());
    Eigen::Quaterniond quat_des = fsm_ut::MatToQuat(rot_des);
}