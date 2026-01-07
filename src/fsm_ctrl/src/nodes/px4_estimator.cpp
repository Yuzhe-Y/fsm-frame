/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-20 14
 * @FilePath: /fsm_ctrl/src/nodes/px4_estimator.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */

#include "fsm_ctrl/nodes/px4_estimator.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_estimator");
    ros::NodeHandle nh("~");

    int mavros_ext_odom_source = 0;
    double mavros_ekf_pos_error_max = 0.1;
    double mavros_ekf_yaw_error_max = 10;
    bool ekf_ready = false;

    /*    parameter    */
    nh.param("/comm_node/mocap_frame", fsm_cb::mocap_frame, 0);
    nh.param("/comm_node/mavros_ext_odom_source", mavros_ext_odom_source, 0);
    nh.param("/comm_node/mavros_ekf_pos_error_max", mavros_ekf_pos_error_max, 0.1);
    nh.param("/comm_node/mavros_ekf_yaw_error_max", mavros_ekf_yaw_error_max, 10.0);

    /*    publisher    */
    ros::Publisher mavros_ext_odom_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/vision_pose/pose", 10);
    ros::Publisher mavros_ext_odom_pose_vel_pub = nh.advertise<nav_msgs::Odometry>
        ("/mavros/odometry/out", 10);
    ros::Publisher mavros_ekf_ready_pub = nh.advertise<std_msgs::Bool>
        ("/mavros/ekf_ready", 10);
    
    /*    subscriber    */
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/vrpn_client_node/UGV7/pose", 10, fsm_cb::MocapOdomCallback);
    ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>
        ("/Odometry", 10, fsm_cb::LidarOdomCallback);    
    ros::Subscriber camera_sub = nh.subscribe<nav_msgs::Odometry>
        ("/Odometry", 10, fsm_cb::CameraOdomCallback);
    ros::Subscriber ext_fcu_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, fsm_cb::ExtFcuPoseCallback);
    
    ros::Rate rate(100.0);
    while(ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::PoseStamped mavros_ext_odom_msg;
        nav_msgs::Odometry mavros_ext_odom_pose_vel_msg;

        // mavros_ext_odom_msg.pose.position.x = 0.0;
        // mavros_ext_odom_msg.pose.position.y = 0.0;
        // mavros_ext_odom_msg.pose.position.z = 0.0;

        // mavros_ext_odom_msg.pose.orientation.x = 0.0;
        // mavros_ext_odom_msg.pose.orientation.y = 0.0;
        // mavros_ext_odom_msg.pose.orientation.z = 0.0;
        // mavros_ext_odom_msg.pose.orientation.w = 1.0;

        // mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::mocap_pos, fsm_cb::mocap_quat);
        // std::cout << "mocap_pos: " << fsm_cb::mocap_pos.transpose() << std::endl;
        // mavros_ext_odom_pub.publish(mavros_ext_odom_msg);
        if(fsm_cb::is_source_new)
        {
            if(mavros_ext_odom_source == 0) // mocap
            {
                mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::mocap_pos, fsm_cb::mocap_quat);
            }
            else if(mavros_ext_odom_source == 1) // lidar
            {
                mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::lidar_pos, fsm_cb::lidar_quat);
                mavros_ext_odom_pose_vel_msg = fsm_ut::SetEkfExtPoseVelData(fsm_cb::lidar_pos, fsm_cb::lidar_vel, fsm_cb::lidar_quat);
            }
            else if(mavros_ext_odom_source == 2) // camera
            {
                mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::camera_pos, fsm_cb::camera_quat);
            }
            else
            {
                ROS_ERROR("Invalid mavros_ext_odom_source: %d", mavros_ext_odom_source);
                continue;
            }
            mavros_ext_odom_msg.header.stamp = ros::Time::now();
            mavros_ext_odom_msg.header.frame_id = "odom";
            mavros_ext_odom_pose_vel_msg.header.stamp = ros::Time::now();
            mavros_ext_odom_pose_vel_msg.header.frame_id = "odom";
            mavros_ext_odom_pose_vel_msg.child_frame_id = "base_link";
            // mavros_ext_odom_pose_vel_pub.publish(mavros_ext_odom_pose_vel_msg);
            mavros_ext_odom_pub.publish(mavros_ext_odom_msg);
            fsm_cb::is_source_new = false;
        }


        Eigen::Quaterniond mavros_ext_odom_vision_quat = Eigen::Quaterniond(mavros_ext_odom_msg.pose.orientation.w, 
                                                                            mavros_ext_odom_msg.pose.orientation.x, 
                                                                            mavros_ext_odom_msg.pose.orientation.y, 
                                                                            mavros_ext_odom_msg.pose.orientation.z);
        Eigen::Vector3d mavros_ext_odom_vision = fsm_ut::QuatToEuler(mavros_ext_odom_vision_quat);
        if(fabs(mavros_ext_odom_msg.pose.position.x - fsm_cb::ext_fcu_pos[0]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_msg.pose.position.y - fsm_cb::ext_fcu_pos[1]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_msg.pose.position.z - fsm_cb::ext_fcu_pos[2]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_vision[2]*180.0/M_PI - fsm_cb::ext_fcu_euler[2]*180.0/M_PI) < mavros_ekf_yaw_error_max)
        {
            ekf_ready = true;
            std_msgs::Bool ekf_msg;
            ekf_msg.data = ekf_ready;
            mavros_ekf_ready_pub.publish(ekf_msg);
        }
        rate.sleep();
    }

    return 0;
}
