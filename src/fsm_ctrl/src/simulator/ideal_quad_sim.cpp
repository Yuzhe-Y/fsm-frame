/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-05 16
 * @FilePath: /fsm_ctrl/src/simulator/ideal_quad_sim.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include <fsm_ctrl/simulator/ideal_quad_sim.hpp>

namespace simulator
{

IdealQuadSim::IdealQuadSim(ros::NodeHandle &nh)
{
    // nh.param("mass", mass, 1.0);
    // inertia.setZero();
    // nh.param("inetia_xy", inertia(0, 0), 1.0);
    // nh.param("inetia_xy", inertia(1, 1), 1.0);
    // nh.param("inetia_z", inertia(2, 2), 1.0);
    // nh.param("arm_len", arm_len, 0.25);
    // nh.param("kf", kf, 5e-8);
    // nh.param("km", km, 1e-9);
    nh.param("hover_thr", hover_thrust_percentage, 0.3);
    update_mode = "position";

    omega_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1, &IdealQuadSim::OmegaCmd_Callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/ideal_quad_sim/odom", 1);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/ideal_quad_sim/imu", 1);  
    state_pub_timer = nh.createTimer(ros::Duration(const_params::SIM_DURATION), &IdealQuadSim::StatePub_Callback, this);
}


void IdealQuadSim::OmegaCmd_Callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    update_mode = "omega";
    now.angular_velocity_cmd = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);
    now.thrust_cmd = msg->thrust;
}


void IdealQuadSim::StatePub_Callback(const ros::TimerEvent &te)
{
    if(update_mode == "position") {PosUpdate();}
    if(update_mode == "omega") {OmegaUpdate();}

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = now.pos.x();
    odom.pose.pose.position.y = now.pos.y();
    odom.pose.pose.position.z = now.pos.z();
    odom.pose.pose.orientation.w = now.quat.w();
    odom.pose.pose.orientation.x = now.quat.x();
    odom.pose.pose.orientation.y = now.quat.y();
    odom.pose.pose.orientation.z = now.quat.z();
    odom.twist.twist.linear.x = now.vel.x();
    odom.twist.twist.linear.y = now.vel.y();
    odom.twist.twist.linear.z = now.vel.z();
    odom.twist.twist.angular.x = now.acc.x();
    odom.twist.twist.angular.y = now.acc.y();
    odom.twist.twist.angular.z = now.acc.z();
    odom_pub.publish(odom);
}


void IdealQuadSim::PosUpdate()
{
    double dt = const_params::SIM_DURATION;
    now.pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    now.vel = (now.pos - last.pos)/dt;
    now.acc = (now.vel - last.vel)/dt;
    Eigen::Matrix3d rot = fsm_ut::DiffFlat(now.acc, now.euler.z());
    now.quat = fsm_ut::MatToQuat(rot).normalized();
    last = now;
}


void IdealQuadSim::OmegaUpdate()
{
    double dt = const_params::SIM_DURATION;
    now.angular_acceleration = (now.angular_velocity - last.angular_velocity)/dt;
    Eigen::Quaterniond dq = Eigen::Quaterniond(1.0, (last.angular_velocity.x() + now.angular_velocity.x())*dt/4, 
                                                    (last.angular_velocity.y() + now.angular_velocity.y())*dt/4, 
                                                    (last.angular_velocity.z() + now.angular_velocity.z())*dt/4);
    now.quat = (last.quat*dq).normalized();
    now.euler = fsm_ut::QuatToEuler(now.quat);
    now.acc = fsm_ut::QuatToMat(now.quat)*Eigen::Vector3d(0.0, 0.0, now.thrust/hover_thrust_percentage*const_params::SIM_GRAVITY) - Eigen::Vector3d(0.0, 0.0, const_params::SIM_GRAVITY);
    now.vel = last.vel + (last.acc + now.acc)*dt/2;
    now.pos = last.pos + (last.vel + now.vel)*dt/2;

    last = now;
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ideal_quad_sim");
    ros::NodeHandle nh;

    simulator::IdealQuadSim ideal_quad_sim(nh);

    ros::spin();

    return 0;
}