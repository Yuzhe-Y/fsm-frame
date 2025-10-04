/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-04 15
 * @FilePath: /fsm_ctrl/include/fsm_ctrl/utils/math_utils/math_utils.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _MATH_UTILS_HPP_
#define _MATH_UTILS_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include "const_params.h"

namespace fsm_ut
{

    class CtrlPt
    {
        public:
            Eigen::Vector3d pos, vel, acc;
            Eigen::Quaterniond quat;
            Eigen::Vector3d euler;
            Eigen::Vector3d rate;
            double thrust;
            Eigen::Vector4d pwm;
    };


    class Thr_LSE
    {
        public:
    
            double p_est, rho;
            double dt;
            double hover_thr, thr;
            double thr_to_acc;
            
            Thr_LSE();
            ~Thr_LSE() = default;
            void Init_ThrEst(double _ctrl_rate, double _hover_thr);
            double LinearThrEst(double _acc);
    };

    class LowPassFilter 
    {
        private:
            double alpha;
            double prevOutput;
            bool firstRun;

        public:
            LowPassFilter();
            LowPassFilter(double cutoffFreq, double samplingFreq); // 构造函数，设置截止频率 cutoffFreq: 截止频率 (Hz) samplingFreq: 采样频率 (Hz)
            void reset(); // 重置滤波器状态
            double filter(double input); // 处理单个数据点
            std::vector<double> filterData(const std::vector<double>& inputs); // 处理一组数据
    };

    class ButterworthFilter 
    {
        public:
            ButterworthFilter();    
            // 构造函数：输入阶数、截止频率（Hz）、采样频率（Hz）
            ButterworthFilter(int order, double cutoff_freq, double sample_freq);
            // 重置状态
            void reset();
            // 实时单点滤波
            double filter(double input_sample);
            // 批量滤波
            std::vector<double> filter(const std::vector<double>& input_signal);
        private:
            int order;

            double cutoff_freq;  // Hz
            double sample_freq;  // Hz

            // 滤波器系数
            double b0, b1, b2;
            double a1, a2;

            // 历史
            double prev_input[2];
            double prev_output[2];

            // 内部设计
            void designFilter();
    };


    Eigen::Vector3d QuatToEuler(double _w, double _x, double _y, double _z);
    Eigen::Vector3d QuatToEuler(Eigen::Quaterniond _quat);

    Eigen::Quaterniond EulerToQuat(double _roll, double _pitch, double _yaw);
    Eigen::Quaterniond EulerToQuat(Eigen::Vector3d _euler);

    Eigen::Matrix3d QuatToMat(double _w, double _x, double _y, double _z);
    Eigen::Matrix3d QuatToMat(Eigen::Quaterniond _quat);

    Eigen::Matrix3d EulerToMat(double _roll, double _pitch, double _yaw);
    Eigen::Matrix3d EulerToMat(Eigen::Vector3d _euler);

    Eigen::Vector3d MatToEuler(Eigen::Matrix3d _mat);
    Eigen::Quaterniond MatToQuat(Eigen::Matrix3d _mat);

    double Clamp(double _data, double _max);
    double Clamp(double _data, double _min, double _max);
    double Sign(double _data);

    int binomialCoefficient(int n, int k);
    std::array<double, 10> transformPolynomial(const std::array<double, 10>& a, double t0);
    std::array<double, 10> reverseTransformPolynomial(const std::array<double, 10>& b, double t0);
    std::array<double, 4> transformSpline(const std::array<double, 4>& a, double s0);
    std::array<double, 4> reverseTransformSpline(const std::array<double, 4>& b, double s0);

    Eigen::Matrix3d DiffFlat(Eigen::Vector3d _acc, double _yaw);
} // namespace fsm_ut
#endif
