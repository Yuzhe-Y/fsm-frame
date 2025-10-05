/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-05 16
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/simulator/ideal_quad_sim.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _IDEAL_QUAD_SIM_HPP_
#define _IDEAL_QUAD_SIM_HPP_

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <fsm_ctrl/utils/math_utils/math_utils.hpp>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <const_params.h>

namespace simulator
{
    struct SimulatorState
    {
        Eigen::Vector3d pos; //位置
        Eigen::Vector3d vel; //速度
        Eigen::Quaterniond quat; //四元数
        Eigen::Vector3d euler; //欧拉角
        Eigen::Vector3d acc; //加速度
        Eigen::Vector3d angular_velocity; //角速度
        Eigen::Vector3d angular_acceleration; //角加速度
        double thrust; //总推力
        
        Eigen::Vector3d angular_velocity_cmd; //角速度指令
        double thrust_cmd; //总推力指令
    };

    class IdealQuadSim
    {
    private:
        std::string update_mode;

        // double mass;
        // Eigen::Matrix3d inertia;
        // double arm_len;
        // double kf, km;
        double hover_thrust_percentage;

        SimulatorState now, last;

        ros::Subscriber omega_cmd_sub;
        ros::Publisher odom_pub, imu_pub;
        ros::Timer state_pub_timer;

    public:
        IdealQuadSim(ros::NodeHandle &nh);
        ~IdealQuadSim() = default;

        void OmegaCmd_Callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
        void StatePub_Callback(const ros::TimerEvent &te);

        void PosUpdate();
        void OmegaUpdate();
    };

}
#endif