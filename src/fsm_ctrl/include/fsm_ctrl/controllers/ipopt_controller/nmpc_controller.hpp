/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-07-15 17
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

#define G 9.8015
#define GRAVITY 9.8015
#define COSDEGREE 0.7071067811865475

class NmpcController
{
    private:
        /*unchanged parameters*/
        const double world_gravity_acc_ = 9.8015; //重力加速度

        /*nlp parameters*/
        double ctrl_T_; //控制器周期
        std::array<double, 2> total_force_limit_; //输入力约束
        std::array<double, 2> w_limit_; //无人机角速度约束
        int nlp_predict_step_; //nlp问题预测步长
        double nlp_onestep_time_; //nlp问题每步预测时间
        int nlp_state_num_; //nlp问题状态变量个数
        int nlp_input_num_; //nlp问题输入变量个数

        Eigen::Matrix<float, 3, 1> nlp_costQ_pos_; //位置代价
        Eigen::Matrix<float, 3, 1> nlp_costQ_vel_; //速度代价
        Eigen::Matrix<float, 3, 1> nlp_costQ_quat_; //姿态代价
        Eigen::Matrix<float, 3, 1> nlp_costR_w_; //角速度代价
        double nlp_costR_totalF_; //输入力导数代价

        /* CASADI basic variables and functions */
        casadi::Function nlp_solver_; //nlp问题求解器
        casadi::Function model_predict_function_; //nlp问题迭代方程
        std::map<std::string, casadi::DM> nlp_result_; //nlp问题最优求解值
        std::map<std::string, casadi::DM> nlp_args_; //nlp求解器参数
        std::vector<double> nlp_initial_u0_; //nlp问题每次开始迭代时输入变量初始值

        /* NMPC outputs */
        Eigen::Vector3d w_command_; //求解结果，最优角速度
        double total_force_command_; //求解结果，最优力
        std::vector<double> optimal_command_; //所有最优输出总集合

    public:
        NmpcController(double ctrl_T, 
                       std::array<double, 2> total_force_limit, 
                       std::array<double, 2> w_limit,
                       int nlp_predict_step, 
                       double nlp_onestep_time, 
                       int nlp_state_num, 
                       int nlp_input_num,
                       Eigen::Matrix<float, 3, 1> nlp_costQ_pos,
                       Eigen::Matrix<float, 3, 1> nlp_costQ_vel,
                       Eigen::Matrix<float, 3, 1> nlp_costQ_quat,
                       Eigen::Matrix<float, 3, 1> nlp_costR_w,
                       double nlp_costR_totalF); //构造函数

        NmpcController(const NmpcController& other); // 拷贝构造函数
        NmpcController& operator=(const NmpcController& other); //赋值运算符重载

        double get_ctrl_T() const {return ctrl_T_;}  //
		std::array<double, 2> get_total_force_limit() const {return total_force_limit_;}  //
		std::array<double, 2> get_w_limit() const {return w_limit_;}  //

        int get_nlpPredictStep() const {return nlp_predict_step_;}  //
		double get_nlp_onestep_time() const {return nlp_onestep_time_;}  //
		int get_nlp_state_num() const {return nlp_state_num_;}  //
		int get_nlp_input_num() const {return nlp_input_num_;}  //

        Eigen::Matrix<float, 3, 1> get_nlp_costQ_pos() const {return nlp_costQ_pos_;}  //
        Eigen::Matrix<float, 3, 1> get_nlp_costQ_vel() const {return nlp_costQ_vel_;}  //
        Eigen::Matrix<float, 3, 1> get_nlp_costQ_quat() const {return nlp_costQ_quat_;}  //
        Eigen::Matrix<float, 3, 1> get_nlp_costR_w() const {return nlp_costR_w_;}  //
        double get_nlp_costR_totalF() const {return nlp_costR_totalF_;}  //

        Eigen::Vector3d get_w_command() const {return w_command_;}  //
        double get_total_force_command() const {return total_force_command_;}  //
        std::vector<double> get_optimal_command() const {return optimal_command_;}  //

        void set_nlp_costQ_pos(Eigen::Matrix<float, 3, 1> nlp_costQ_pos) {nlp_costQ_pos_ = nlp_costQ_pos;}  //
        void set_nlp_costQ_vel(Eigen::Matrix<float, 3, 1> nlp_costQ_vel) {nlp_costQ_vel_ = nlp_costQ_vel;}  //
        void set_nlp_costQ_quat(Eigen::Matrix<float, 3, 1> nlp_costQ_quat) {nlp_costQ_quat_ = nlp_costQ_quat;}  //
        void set_nlp_costR_w(Eigen::Matrix<float, 3, 1> nlp_costR_w) {nlp_costR_w_ = nlp_costR_w;}  //
        void set_nlp_costR_totalF(double nlp_costR_totalF) {nlp_costR_totalF_ = nlp_costR_totalF;}  // 
        
        void OptimalSolution(std::vector<double> current_states, std::vector<double> desired_params);
};

#endif