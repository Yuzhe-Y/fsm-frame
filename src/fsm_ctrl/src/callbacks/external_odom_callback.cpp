/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-02 11
 * @FilePath: /fsm_ctrl/src/callbacks/external_odom_callback.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include "fsm_ctrl/callbacks/external_odom_callback.hpp"

namespace fsm_cb
{
    bool is_mocap_init = false;
    int mocap_frame = 0;
    Eigen::Vector3d mocap_pos;                   
    Eigen::Quaterniond mocap_quat;
    Eigen::Vector3d mocap_euler;
    Eigen::Vector3d mocap_pos_init;                   
    Eigen::Quaterniond mocap_quat_init;

    Eigen::Vector3d lidar_pos;
    Eigen::Vector3d lidar_vel;                     
    Eigen::Quaterniond lidar_quat;
    Eigen::Vector3d lidar_euler; 

    Eigen::Vector3d camera_pos;
    Eigen::Vector3d camera_vel;                     
    Eigen::Quaterniond camera_quat;
    Eigen::Vector3d camera_euler;

    Eigen::Vector3d ext_fcu_pos;
    Eigen::Vector3d ext_fcu_vel;                     
    Eigen::Quaterniond ext_fcu_quat;
    Eigen::Vector3d ext_fcu_euler;

    ros::Time ext_odom_stamp;
    bool is_source_new = false; //数据更新标志

    /**
     * @description: 
     * @param {ConstPtr} &msg
     * @return {*}
     */
    void MocapOdomCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ext_odom_stamp = msg->header.stamp;

        if(mocap_frame == 0)
        {
            if(!is_mocap_init)
            {
                mocap_pos_init = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                mocap_quat_init = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
                is_mocap_init = true;
            }
            
            Eigen::Vector3d pos_raw = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Eigen::Quaterniond quat_raw = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            mocap_pos = mocap_quat_init.inverse()*(pos_raw - mocap_pos_init);
            mocap_quat = mocap_quat_init.inverse()*quat_raw;
            mocap_euler = fsm_ut::QuatToEuler(mocap_quat);
            is_source_new = true;
        }
    
        else if(mocap_frame == 1)
        {
            if(!is_mocap_init)
            {
                mocap_pos_init = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y);
                mocap_quat_init = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y);
                is_mocap_init = true;
            }
    
            Eigen::Vector3d pos_raw = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y);
            Eigen::Quaterniond quat_raw = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y);
            mocap_pos = mocap_quat_init.inverse()*(pos_raw - mocap_pos_init);
            mocap_quat = mocap_quat_init.inverse()*quat_raw;
            mocap_euler = fsm_ut::QuatToEuler(mocap_quat);
            is_source_new = true;
        }
    
        else
        {
            ROS_ERROR("Invalid Motive Frame!!!");
            return;
        }
    
    }

    /**
     * @description: 
     * @param {ConstPtr} &msg
     * @return {*}
     */
    void LidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ext_odom_stamp = msg->header.stamp;
        lidar_pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        lidar_vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        lidar_quat = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        lidar_euler = fsm_ut::QuatToEuler(lidar_quat);
        is_source_new = true;
    }

    /**
     * @description: 
     * @param {ConstPtr} &msg
     * @return {*}
     */
    void CameraOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ext_odom_stamp = msg->header.stamp;
        camera_pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        camera_vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        camera_quat = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        camera_euler = fsm_ut::QuatToEuler(camera_quat);
        is_source_new = true;
    }

    /**
     * @description: 
     * @param {geometry_msgs::PoseStamped::ConstPtr} &msg
     * @return {*}
     */
    void ExtFcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ext_fcu_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        ext_fcu_quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        ext_fcu_euler = fsm_ut::QuatToEuler(ext_fcu_quat);
    }
}