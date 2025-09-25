/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-24 19
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/controllers/ipopt_controller/nmpc_controller.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef __NMPC_CONTROLLER_H__
#define __NMPC_CONTROLLER_H__

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "fsm_ctrl/utils/math_utils/math_utils.hpp"
#include "const_params.h"

class NMPC_Ctrller_df
{
    private:
        /*unchanged parameters*/
        const double drone_mass = const_params::DRONE_MASS; //无人机质量0.9
        const double world_gravity_acc = const_params::GRAVITY; //重力加速度
        const double drone_arm_L = const_params::DRONE_ARM_L; //无人机机臂长度0.16
        const double drone_cm = const_params::DRONE_CM; //电机反扭力系数4.888486266072161e-06
        const double drone_ct = const_params::DRONE_CT; //电机推力系数0.000367717
        const double drone_cq = drone_cm/drone_ct; //中间系数，用于映射矩阵
        const Eigen::Vector3d drone_MomentOfInertia = Eigen::Vector3d(const_params::DRONE_INERTIA_XX, 
                                                                      const_params::DRONE_INERTIA_YY, 
                                                                      const_params::DRONE_INERTIA_ZZ); //无人机旋转惯量
        const Eigen::Vector3d drone_Inv_MomentOfInertia = drone_MomentOfInertia.cwiseInverse(); //无人机旋转变量倒数

        /*NLP parameters*/
        double ctrl_T; //控制器周期
        std::array<double, 2> force_limit; //输入力约束
        std::array<double, 2> delta_force_limit; //输入力导数约束
        std::array<double, 2> w_limit; //无人机角速度约束
        int NLP_predict_step; //NLP问题预测步长
        double NLP_onestep_time; //NLP问题每步预测时间
        int NLP_state_num; //NLP问题状态变量个数
        int NLP_input_num; //NLP问题输入变量个数

        Eigen::Matrix<float, 3, 1> NLP_costQ_pos; //位置代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_vel; //速度代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_quat; //姿态代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_w; //角速度代价
        Eigen::Matrix<float, 4, 1> NLP_costRdf; //输入力导数代价

        /* CASADI basic variables and functions */
        casadi::Function NLP_solver; //NLP问题求解器
        casadi::Function model_predict_function; //NLP问题迭代方程
        std::map<std::string, casadi::DM> NLP_result; //NLP问题最优求解值
        std::map<std::string, casadi::DM> NLP_args; //NLP求解器参数
        std::vector<double> NLP_initial_u0; //NLP问题每次开始迭代时输入变量初始值

        /* NMPC outputs */
        Eigen::Vector4d drone_delta_force_command; //求解结果，最优力
        std::vector<double> optimal_command; //所有最优输出总集合

    public:
        NMPC_Ctrller_df(double _ctrl_T, 
                        std::array<double, 2> _force_limit, 
                        std::array<double, 2> _delta_force_limit, 
                        std::array<double, 2> _w_limit,
                        int _NLP_predict_step, 
                        double _NLP_onestep_time, 
                        int _NLP_state_num, 
                        int _NLP_input_num,
                        Eigen::Matrix<float, 3, 1> _NLP_costQ_pos,
                        Eigen::Matrix<float, 3, 1> _NLP_costQ_vel,
                        Eigen::Matrix<float, 3, 1> _NLP_costQ_quat,
                        Eigen::Matrix<float, 3, 1> _NLP_costQ_w,
                        Eigen::Matrix<float, 4, 1> _NLP_costRdf); //构造函数

        NMPC_Ctrller_df(const NMPC_Ctrller_df& other); // 拷贝构造函数
        NMPC_Ctrller_df& operator=(const NMPC_Ctrller_df& other); //赋值运算符重载

        double getCtrlT() const {return ctrl_T;}  //
		std::array<double, 2> getForceLimit() const {return force_limit;}  //
		std::array<double, 2> getDeltaForceLimit() const {return delta_force_limit;}  //
		std::array<double, 2> getWLimit() const {return w_limit;}  //

        int getNLPPredictStep() const {return NLP_predict_step;}  //
		double getNLPOnestepTime() const {return NLP_onestep_time;}  //
		int getNLPStateNum() const {return NLP_state_num;}  //
		int getNLPInputNum() const {return NLP_input_num;}  //

        Eigen::Matrix<float, 3, 1> getNLPcostQpos() const {return NLP_costQ_pos;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQvel() const {return NLP_costQ_vel;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQquat() const {return NLP_costQ_quat;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQw() const {return NLP_costQ_w;}  //
        Eigen::Matrix<float, 4, 1> getNLPcostRdf() const {return NLP_costRdf;}  //

        Eigen::Vector4d getDroneDForceCommand() const {return drone_delta_force_command;}  //
        std::vector<double> getOptimalCommand() const {return optimal_command;}  //

        void setNLPCostQpos(Eigen::Matrix<float, 3, 1> _NLP_costQpos) {NLP_costQ_pos = _NLP_costQpos;}  //
        void setNLPCostQvel(Eigen::Matrix<float, 3, 1> _NLP_costQvel) {NLP_costQ_vel = _NLP_costQvel;}  //
        void setNLPCostQquat(Eigen::Matrix<float, 3, 1> _NLP_costQquat) {NLP_costQ_quat = _NLP_costQquat;}  //
        void setNLPCostQw(Eigen::Matrix<float, 3, 1> _NLP_costQw) {NLP_costQ_w = _NLP_costQw;}  //
        void setNLPCostRdf(Eigen::Matrix<float, 4, 1> _NLP_costRdf) {NLP_costRdf = _NLP_costRdf;}  //
        
        void optimal_solution(std::vector<double> _current_states, std::vector<double> _desired_params);
};

class NMPC_Ctrller_f
{
    private:
        /*unchanged parameters*/
        const double drone_mass = const_params::DRONE_MASS; //无人机质量0.9
        const double world_gravity_acc = const_params::GRAVITY; //重力加速度
        const double drone_arm_L = const_params::DRONE_ARM_L; //无人机机臂长度0.16
        const double drone_cm = const_params::DRONE_CM; //电机反扭力系数4.888486266072161e-06
        const double drone_ct = const_params::DRONE_CT; //电机推力系数0.000367717
        const double drone_cq = drone_cm/drone_ct; //中间系数，用于映射矩阵
        const Eigen::Vector3d drone_MomentOfInertia = Eigen::Vector3d(const_params::DRONE_INERTIA_XX, 
                                                                      const_params::DRONE_INERTIA_YY, 
                                                                      const_params::DRONE_INERTIA_ZZ); //无人机旋转惯量
        const Eigen::Vector3d drone_Inv_MomentOfInertia = drone_MomentOfInertia.cwiseInverse(); //无人机旋转变量倒数

        /*NLP parameters*/
        double ctrl_T; //控制器周期
        std::array<double, 2> force_limit; //输入力约束
        std::array<double, 2> w_limit; //无人机角速度约束
        int NLP_predict_step; //NLP问题预测步长
        double NLP_onestep_time; //NLP问题每步预测时间
        int NLP_state_num; //NLP问题状态变量个数
        int NLP_input_num; //NLP问题输入变量个数

        Eigen::Matrix<float, 3, 1> NLP_costQ_pos; //位置代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_vel; //速度代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_quat; //姿态代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_w; //角速度代价
        Eigen::Matrix<float, 4, 1> NLP_costRf; //输入力导数代价

        /* CASADI basic variables and functions */
        casadi::Function NLP_solver; //NLP问题求解器
        casadi::Function model_predict_function; //NLP问题迭代方程
        std::map<std::string, casadi::DM> NLP_result; //NLP问题最优求解值
        std::map<std::string, casadi::DM> NLP_args; //NLP求解器参数
        std::vector<double> NLP_initial_u0; //NLP问题每次开始迭代时输入变量初始值

        /* NMPC outputs */
        Eigen::Vector4d drone_force_command; //求解结果，最优力
        std::vector<double> optimal_command; //所有最优输出总集合

    public:
        NMPC_Ctrller_f(double _ctrl_T, 
                       std::array<double, 2> _force_limit, 
                       std::array<double, 2> _w_limit,
                       int _NLP_predict_step, 
                       double _NLP_onestep_time, 
                       int _NLP_state_num, 
                       int _NLP_input_num,
                       Eigen::Matrix<float, 3, 1> _NLP_costQ_pos,
                       Eigen::Matrix<float, 3, 1> _NLP_costQ_vel,
                       Eigen::Matrix<float, 3, 1> _NLP_costQ_quat,
                       Eigen::Matrix<float, 3, 1> _NLP_costQ_w,
                       Eigen::Matrix<float, 4, 1> _NLP_costRf); //构造函数

        NMPC_Ctrller_f(const NMPC_Ctrller_f& other); // 拷贝构造函数
        NMPC_Ctrller_f& operator=(const NMPC_Ctrller_f& other); //赋值运算符重载

        double getCtrlT() const {return ctrl_T;}  //
		std::array<double, 2> getForceLimit() const {return force_limit;}  //
		std::array<double, 2> getWLimit() const {return w_limit;}  //

        int getNLPPredictStep() const {return NLP_predict_step;}  //
		double getNLPOnestepTime() const {return NLP_onestep_time;}  //
		int getNLPStateNum() const {return NLP_state_num;}  //
		int getNLPInputNum() const {return NLP_input_num;}  //

        Eigen::Matrix<float, 3, 1> getNLPcostQpos() const {return NLP_costQ_pos;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQvel() const {return NLP_costQ_vel;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQquat() const {return NLP_costQ_quat;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQw() const {return NLP_costQ_w;}  //
        Eigen::Matrix<float, 4, 1> getNLPcostRf() const {return NLP_costRf;}  //

        Eigen::Vector4d getDroneForceCommand() const {return drone_force_command;}  //
        std::vector<double> getOptimalCommand() const {return optimal_command;}  //

        void setNLPCostQpos(Eigen::Matrix<float, 3, 1> _NLP_costQpos) {NLP_costQ_pos = _NLP_costQpos;}  //
        void setNLPCostQvel(Eigen::Matrix<float, 3, 1> _NLP_costQvel) {NLP_costQ_vel = _NLP_costQvel;}  //
        void setNLPCostQquat(Eigen::Matrix<float, 3, 1> _NLP_costQquat) {NLP_costQ_quat = _NLP_costQquat;}  //
        void setNLPCostQw(Eigen::Matrix<float, 3, 1> _NLP_costQw) {NLP_costQ_w = _NLP_costQw;}  //
        void setNLPCostRf(Eigen::Matrix<float, 4, 1> _NLP_costRf) {NLP_costRf = _NLP_costRf;}  // 

        void optimal_solution(std::vector<double> _current_states, std::vector<double> _desired_params);
};

class NMPC_Ctrller_simple
{
    private:
        /*unchanged parameters*/
        const double drone_mass = const_params::DRONE_MASS; //无人机质量0.9
        const double world_gravity_acc = const_params::GRAVITY; //重力加速度
        const double drone_arm_L = const_params::DRONE_ARM_L; //无人机机臂长度0.16

        /*NLP parameters*/
        double ctrl_T; //控制器周期
        std::array<double, 2> total_force_limit; //输入力约束
        std::array<double, 2> w_limit; //无人机角速度约束
        int NLP_predict_step; //NLP问题预测步长
        double NLP_onestep_time; //NLP问题每步预测时间
        int NLP_state_num; //NLP问题状态变量个数
        int NLP_input_num; //NLP问题输入变量个数

        Eigen::Matrix<float, 3, 1> NLP_costQ_pos; //位置代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_vel; //速度代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_quat; //姿态代价
        Eigen::Matrix<float, 3, 1> NLP_costR_w; //角速度代价
        double NLP_costR_totalF; //输入力导数代价

        /* CASADI basic variables and functions */
        casadi::Function NLP_solver; //NLP问题求解器
        casadi::Function model_predict_function; //NLP问题迭代方程
        std::map<std::string, casadi::DM> NLP_result; //NLP问题最优求解值
        std::map<std::string, casadi::DM> NLP_args; //NLP求解器参数
        std::vector<double> NLP_initial_u0; //NLP问题每次开始迭代时输入变量初始值

        /* NMPC outputs */
        Eigen::Vector3d w_command; //求解结果，最优角速度
        double total_force_command; //求解结果，最优力
        std::vector<double> optimal_command; //所有最优输出总集合

    public:
        NMPC_Ctrller_simple(double _ctrl_T, 
                            std::array<double, 2> _total_force_limit, 
                            std::array<double, 2> _w_limit,
                            int _NLP_predict_step, 
                            double _NLP_onestep_time, 
                            int _NLP_state_num, 
                            int _NLP_input_num,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_pos,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_vel,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_quat,
                            Eigen::Matrix<float, 3, 1> _NLP_costR_w,
                            double _NLP_costR_totalF); //构造函数
        NMPC_Ctrller_simple(); //默认构造函数

        NMPC_Ctrller_simple(const NMPC_Ctrller_simple& other); // 拷贝构造函数
        NMPC_Ctrller_simple& operator=(const NMPC_Ctrller_simple& other); //赋值运算符重载

        double getCtrlT() const {return ctrl_T;}  //
		std::array<double, 2> getTotalForceLimit() const {return total_force_limit;}  //
		std::array<double, 2> getWLimit() const {return w_limit;}  //

        int getNLPPredictStep() const {return NLP_predict_step;}  //
		double getNLPOnestepTime() const {return NLP_onestep_time;}  //
		int getNLPStateNum() const {return NLP_state_num;}  //
		int getNLPInputNum() const {return NLP_input_num;}  //

        Eigen::Matrix<float, 3, 1> getNLPcostQpos() const {return NLP_costQ_pos;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQvel() const {return NLP_costQ_vel;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQquat() const {return NLP_costQ_quat;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostRw() const {return NLP_costR_w;}  //
        double getNLPcostRtotalF() const {return NLP_costR_totalF;}  //

        Eigen::Vector3d getwCommand() const {return w_command;}  //
        double getTotalFCommand() const {return total_force_command;}  //
        std::vector<double> getOptimalCommand() const {return optimal_command;}  //

        void setNLPCostQpos(Eigen::Matrix<float, 3, 1> _NLP_costQpos) {NLP_costQ_pos = _NLP_costQpos;}  //
        void setNLPCostQvel(Eigen::Matrix<float, 3, 1> _NLP_costQvel) {NLP_costQ_vel = _NLP_costQvel;}  //
        void setNLPCostQquat(Eigen::Matrix<float, 3, 1> _NLP_costQquat) {NLP_costQ_quat = _NLP_costQquat;}  //
        void setNLPCostRw(Eigen::Matrix<float, 3, 1> _NLP_costRw) {NLP_costR_w = _NLP_costRw;}  //
        void setNLPCostRtotalF(double _NLP_costR_totalF) {NLP_costR_totalF = _NLP_costR_totalF;}  // 
        
        void optimal_solution(std::vector<double> _current_states, std::vector<double> _desired_params);
};

#endif