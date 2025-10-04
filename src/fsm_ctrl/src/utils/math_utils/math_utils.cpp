/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-04 15
 * @FilePath: /fsm_ctrl/src/utils/math_utils/math_utils.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

namespace fsm_ut
{

/**
 * @brief  convert quaternion to euler angle
 * @param  _w, _x, _y, _z: quaternion
 * @return euler angle
 */
Eigen::Vector3d QuatToEuler(double _w, double _x, double _y, double _z)
{
    Eigen::Vector3d euler;

    double sinr = 2.0*(_w*_x + _y*_z);
    double cosr = 1.0 - 2.0*(_x*_x + _y*_y);
    euler[0] = std::atan2(sinr, cosr);

    double sinp = 2.0*(_w*_y - _z*_x);
    if(std::abs(sinp) >= 1) {euler[1] = std::copysign(M_PI/2, sinp);}
    else                    {euler[1] = std::asin(sinp);}

    double siny = 2.0*(_w*_z + _x*_y);
    double cosy = 1.0 - 2.0*(_y*_y + _z*_z);
    euler[2] = std::atan2(siny, cosy);
    
    return euler;
}

Eigen::Vector3d QuatToEuler(Eigen::Quaterniond _quat)
{
    Eigen::Vector3d euler = QuatToEuler(_quat.w(), _quat.x(), _quat.y(), _quat.z());
    return euler;
}


/**
 * @brief  convert euler angle to quaternion 
 * @param  _roll, _pitch, _yaw: euler angle
 * @return quaternion
 */
Eigen::Quaterniond EulerToQuat(double _roll, double _pitch, double _yaw)
{
    Eigen::Quaterniond quat;
    double cr = std::cos(_roll*0.5);
    double sr = std::sin(_roll*0.5);
    double cp = std::cos(_pitch*0.5);
    double sp = std::sin(_pitch*0.5);
    double cy = std::cos(_yaw*0.5);
    double sy = std::sin(_yaw*0.5);
    quat.w() = cr*cp*cy + sr*sp*sy;
    quat.x() = sr*cp*cy - cr*sp*sy;
    quat.y() = cr*sp*cy + sr*cp*sy;
    quat.z() = cr*cp*sy - sr*sp*cy;
    return quat;
}

Eigen::Quaterniond EulerToQuat(Eigen::Vector3d _euler)
{
    Eigen::Quaterniond quat = EulerToQuat(_euler[0], _euler[1], _euler[2]);
    return quat;
}


/**
 * @brief  convert quaternion to rotation matrix
 * @param  _w, _x, _y, _z: quaternion
 * @return rotation matrix
 */
Eigen::Matrix3d QuatToMat(double _w, double _x, double _y, double _z)
{
    Eigen::Matrix3d mat;
    mat << _w*_w + _x*_x - _y*_y - _z*_z, 2*(_x*_y - _w*_z), 2*(_x*_z + _w*_y),
           2*(_x*_y + _w*_z), _w*_w - _x*_x + _y*_y - _z*_z, 2*(_y*_z - _w*_x),
           2*(_x*_z - _w*_y), 2*(_y*_z + _w*_x), _w*_w - _x*_x - _y*_y + _z*_z;
    return mat;
}

Eigen::Matrix3d QuatToMat(Eigen::Quaterniond _quat)
{
    Eigen::Matrix3d mat;
    mat = QuatToMat(_quat.w(), _quat.x(), _quat.y(), _quat.z());
    return mat;
}


/**
 * @brief  convert euler angle to rotation matrix
 * @param  _roll, _pitch, _yaw: euler angle
 * @return rotation matrix
 */
Eigen::Matrix3d EulerToMat(double _roll, double _pitch, double _yaw)
{
    Eigen::Matrix3d mat_r, mat_p, mat_y;
    mat_r << 1.0,    0.0,         0.0,
             0.0, cos(_roll), -sin(_roll),
             0.0, sin(_roll),  cos(_roll);
    mat_p << cos(_pitch), 0.0, sin(_pitch),
                0.0,      1.0,    0.0,
            -sin(_pitch), 0.0, cos(_pitch);
    mat_y << cos(_yaw), -sin(_yaw), 0.0,
             sin(_yaw),  cos(_yaw), 0.0,
                0.0,        0.0,    1.0;
    Eigen::Matrix3d mat = mat_y*mat_p*mat_r;
    return mat;
}

Eigen::Matrix3d EulerToMat(Eigen::Vector3d _euler)
{
    Eigen::Matrix3d mat = EulerToMat(_euler[0], _euler[1], _euler[2]);
    return mat;
}


/**
 * @brief  convert rotation matrix to euler angle
 * @param  _mat: rotation matrix
 * @return euler angle
 */
Eigen::Vector3d MatToEuler(Eigen::Matrix3d _mat)
{
    Eigen::Vector3d euler;
    euler[0] = atan2(_mat(2,1), _mat(2,2));
    euler[1] = asin(-_mat(2,0));
    euler[2] = atan2(_mat(1,0), _mat(0,0));
    return euler;
}


/**
 * @brief  convert rotation matrix to quaternion
 * @param  _mat: rotation matrix
 * @return quaternion
 */
Eigen::Quaterniond MatToQuat(Eigen::Matrix3d _mat)
{
    Eigen::Quaterniond quat;
    quat.w() = sqrt(1.0 + _mat(0,0) + _mat(1,1) + _mat(2,2))*0.5;
    quat.x() = ( _mat(2,1) - _mat(1,2) )/(4*quat.w());
    quat.y() = ( _mat(0,2) - _mat(2,0) )/(4*quat.w());
    quat.z() = ( _mat(1,0) - _mat(0,1) )/(4*quat.w());
    return quat;
}


/**
 * @brief  clamp with single limit 
 * @param  _data: raw value
 * @param  _max: limit
 * @return clamped value
 */
double Clamp(double _data, double _max)
{
    if(_data < -_max) {return -_max;}
    if(_data > _max) {return _max;}
    return _data;
}


/**
 * @brief  clamp with double limit 
 * @param  _data: raw value
 * @param  _min, _max: lower & upper limit
 * @return clamped value
 */
double Clamp(double _data, double _min, double _max)
{
    if(_data < _min) {return _min;}
    if(_data > _max) {return _max;}
    return _data;
}


/**
 * @brief  sign function
 * @param  _data: raw value
 * @return sign of the value
 */
double Sign(double _data)
{
    if(_data > 0.0) {return 1.0;}
    if(_data < 0.0) {return -1.0;}
    return 0.0;
}


/**
 * @fn     Thr_LSE
 * @brief  constructer of Thr_LSE
 * @param  NULL 
 * @return NULL
 */
Thr_LSE::Thr_LSE()
{
    p_est = 10.0;
    rho = 0.998;         // forgetting factor, Do Not Change!!!
    dt = 0.0;
    hover_thr = 0.0;
    thr = 0.0;
    thr_to_acc = static_cast<double>(INFINITY);
}


/**
 * @fn     Init_ThrEst
 * @brief  initialize thrust estimator
 * @param  _rate: control rate
 * @param  _hover_thr: hover thrust
 * @return NULL
 */
void Thr_LSE::Init_ThrEst(double _rate, double _hover_thr)
{
    dt = 1.0/_rate;
    hover_thr = _hover_thr;
    thr = _hover_thr;
    thr_to_acc = const_params::GRAVITY/_hover_thr;
}


/**
 * @fn     LinearThrEst
 * @brief  recursive least squares thrust estimation with forgetting factor
 * @param  _acc: acceleration
 * @return thrust
 * @note   model: acc = thr_to_acc * thr, thr ∈ (0, 1)
 */
double Thr_LSE::LinearThrEst(double _acc)
{   
    double gamma = 1.0/(rho + thr*p_est*thr);
    double k_est = gamma*p_est*thr;
    thr_to_acc = thr_to_acc + k_est*(_acc - thr*thr_to_acc);
    p_est = (1.0 - k_est*thr)*p_est/rho;

    /*    esitimation divergence    */
    if(abs(const_params::GRAVITY/hover_thr - thr_to_acc)/(const_params::GRAVITY/hover_thr) > 0.2)
    {
        ROS_ERROR("Thrust Estimation Diverge !!!");
        thr_to_acc = const_params::GRAVITY/hover_thr;    // reset coefficient    
    }
    
    thr = _acc/thr_to_acc;
    
    if(thr < 0.1 || thr > 0.9)
    {
        ROS_ERROR("Thrust Esitimation Error !!! Thrust Out of Linear Range !!!");
        // return hover_thr;
        return thr;
    }
    else {return thr;}
}

LowPassFilter::LowPassFilter() 
    : alpha(0.0), prevOutput(0.0), firstRun(true) 
{
    // 默认构造函数，不执行任何初始化逻辑
}

LowPassFilter::LowPassFilter(double cutoffFreq, double samplingFreq) 
{
    // 计算时间常数
    double dt = 1.0 / samplingFreq;
    double rc = 1.0 / (2 * M_PI * cutoffFreq);
    alpha = dt / (rc + dt);
    
    prevOutput = 0.0;
    firstRun = true;
}


void LowPassFilter::reset() 
{
    prevOutput = 0.0;
    firstRun = true;
}


double LowPassFilter::filter(double input) 
{
    if (firstRun) 
    {
        prevOutput = input;
        firstRun = false;
        return input;
    }
    // 一阶低通滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
    double output = alpha * input + (1 - alpha) * prevOutput;
    prevOutput = output;
    return output;
}


std::vector<double> LowPassFilter::filterData(const std::vector<double>& inputs) 
{
    std::vector<double> outputs;
    outputs.reserve(inputs.size());

    for (const double& input : inputs) 
    {outputs.push_back(filter(input));}

    return outputs;
}

ButterworthFilter::ButterworthFilter()
    : order(0), cutoff_freq(0.0), sample_freq(0.0), b0(0.0), b1(0.0), b2(0.0), a1(0.0), a2(0.0) {
    // 空构造函数，不执行滤波器设计，仅初始化成员变量
    prev_input[0] = prev_input[1] = 0.0;
    prev_output[0] = prev_output[1] = 0.0;
}

ButterworthFilter::ButterworthFilter(int order, double cutoff_freq, double sample_freq)
    : order(order), cutoff_freq(cutoff_freq), sample_freq(sample_freq) {
    designFilter();
    reset();
}

void ButterworthFilter::reset() {
    prev_input[0] = prev_input[1] = 0.0;
    prev_output[0] = prev_output[1] = 0.0;
}

void ButterworthFilter::designFilter() {
    double omega_c = 2.0 * M_PI * cutoff_freq; // rad/s
    double T = 1.0 / sample_freq;             // 采样周期
    double omega_s = 2.0 / T;                 // 双线性变换映射

    if (order == 1) {
        // 一阶 Butterworth
        double C = omega_s / omega_c;
        double denom = C + 1.0;

        b0 = 1.0 / denom;
        b1 = b0;
        b2 = 0.0;

        a1 = (C - 1.0) / denom;
        a2 = 0.0;

    } else if (order == 2) {
        // 二阶 Butterworth
        double C = std::tan(M_PI * cutoff_freq / sample_freq);
        double C2 = C * C;
        double sqrt2 = std::sqrt(2.0);

        double denom = 1.0 + sqrt2 * C + C2;

        b0 = C2 / denom;
        b1 = 2.0 * b0;
        b2 = b0;

        a1 = 2.0 * (C2 - 1.0) / denom;
        a2 = (1.0 - sqrt2 * C + C2) / denom;
    }
}

double ButterworthFilter::filter(double input_sample) {
    double output_sample = 0.0;

    if (order == 1) {
        output_sample = b0 * input_sample
                      + b1 * prev_input[0]
                      - a1 * prev_output[0];

        prev_input[0] = input_sample;
        prev_output[0] = output_sample;

    } else if (order == 2) {
        output_sample = b0 * input_sample
                      + b1 * prev_input[0]
                      + b2 * prev_input[1]
                      - a1 * prev_output[0]
                      - a2 * prev_output[1];

        prev_input[1] = prev_input[0];
        prev_input[0] = input_sample;

        prev_output[1] = prev_output[0];
        prev_output[0] = output_sample;
    }

    return output_sample;
}

std::vector<double> ButterworthFilter::filter(const std::vector<double>& input_signal) {
    std::vector<double> output_signal;
    output_signal.reserve(input_signal.size());

    for (size_t n = 0; n < input_signal.size(); ++n) {
        output_signal.push_back(filter(input_signal[n]));
    }

    return output_signal;
}


// 计算二项式系数 C(n, k) = n! / (k! * (n-k)!)
int binomialCoefficient(int n, int k) 
{
    if (k > n) 
        return 0;
    int result = 1;
    for (int i = 0; i < k; ++i) 
    {
        result *= (n - i);
        result /= (i + 1);
    }
    return result;
}


// 变换多项式，使其从标准幂次变为泰勒展开形式，以 (t - t0) 为变量
std::array<double, 10> transformPolynomial(const std::array<double, 10>& a, double t0) 
{
    const int n = a.size() - 1;  // 最高阶数
    std::array<double, 10> b = {0.0};  // 存储变换后的系数，初始化为0

    // 计算新的系数 b_m
    for (int k = 0; k <= n; ++k) 
    {  // 遍历原多项式每一项 a_k * t^k
        for (int m = 0; m <= k; ++m) 
        {  // 展开 (t - t0)^k，计算 b_m
            b[m] += a[k] * binomialCoefficient(k, m) * pow(t0, k - m);
        }
    }
    return b;
}


// 逆变换：将泰勒展开形式，以 (t - t0) 为变量的多项式系数转换回标准幂次， t^n 形式
std::array<double, 10> reverseTransformPolynomial(const std::array<double, 10>& b, double t0) 
{
    const int n = b.size() - 1;  // 最高阶数
    std::array<double, 10> a = {0.0};  // 存储转换回来的系数，初始化为0

    // 计算新的系数 a_k
    for (int m = 0; m <= n; ++m) 
    {  // 遍历每一项 b_m * (t - t0)^m
        for (int k = 0; k <= m; ++k) 
        {  // 展开 (t - t0)^m 为 t^k
            a[k] += b[m] * binomialCoefficient(m, k) * pow(-t0, m - k);
        }
    }
    return a;
}

// 变换样条曲线，使其从标准幂次变为泰勒展开形式，以 (s - s0) 为变量
std::array<double, 4> transformSpline(const std::array<double, 4>& a, double s0) 
{
    const int n = a.size() - 1;  // 最高阶数
    std::array<double, 4> b = {0.0};  // 存储变换后的系数，初始化为0

    // 计算新的系数 b_m
    for (int k = 0; k <= n; ++k) 
    {  // 遍历原多项式每一项 a_k * t^k
        for (int m = 0; m <= k; ++m) 
        {  // 展开 (s - s0)^k，计算 b_m
            b[m] += a[k] * binomialCoefficient(k, m) * pow(s0, k - m);
        }
    }
    return b;
}

// 逆变换：将泰勒展开形式，以 (s - s0) 为变量的样条系数转换回标准幂次， s^n 形式
std::array<double, 4> reverseTransformSpline(const std::array<double, 4>& b, double s0) 
{
    const int n = b.size() - 1;  // 最高阶数
    std::array<double, 4> a = {0.0};  // 存储转换回来的系数，初始化为0

    // 计算新的系数 a_k
    for (int m = 0; m <= n; ++m) 
    {  // 遍历每一项 b_m * (s - s0)^m
        for (int k = 0; k <= m; ++k) 
        {  // 展开 (s - s0)^m 为 s^k
            a[k] += b[m] * binomialCoefficient(m, k) * pow(-s0, m - k);
        }
    }
    return a;
}

/**
 * @fn     DiffFlat
 * @brief  differential flatness transformation
 * @param  _acc_thr: acceleration from thrust
 * @param  _yaw: yaw angle
 * @return rotation matrix
 */
Eigen::Matrix3d DiffFlat(Eigen::Vector3d _acc_thr, double _yaw)
{
    Eigen::Vector3d zB = _acc_thr.normalized();
    Eigen::Vector3d yC = Eigen::Vector3d(-std::sin(_yaw), std::cos(_yaw), 0.0);
    Eigen::Vector3d xB = yC.cross(zB).normalized();
    Eigen::Vector3d yB = zB.cross(xB);
    Eigen::Matrix3d mat;
    mat << xB, yB, zB;
    return mat;
}

} // namespace fsm_ut
