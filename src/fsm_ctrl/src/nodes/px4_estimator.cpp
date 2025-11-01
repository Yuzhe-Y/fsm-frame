/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-24 22
 * @FilePath: /fsm_ctrl/src/nodes/px4_estimator.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */

#include "fsm_ctrl/nodes/px4_estimator.hpp"

using namespace std;

static std_msgs::Bool ready_to_start; //ros消息，融合结束允许起飞标志

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_estimator");
    ros::NodeHandle nh("~");

    int mavros_ext_odom_source = 0;
    double mavros_ekf_pos_error_max = 0.1;
    double mavros_ekf_yaw_error_max = 10;

    /*    parameter    */
    nh.param("mocap_frame", fsm_cb::mocap_frame, 0);
    nh.param("mavros_ext_odom_source", mavros_ext_odom_source, 0);
    nh.param("mavros_ekf_pos_error_max", mavros_ekf_pos_error_max, 0.1);
    nh.param("mavros_ekf_yaw_error_max", mavros_ekf_yaw_error_max, 10.0);

    /*    publisher    */
    ros::Publisher mavros_ext_odom_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/vision_pose/pose", 10);
    
    /*    subscriber    */
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/vrpn_client_node/drone0/pose", 10, fsm_cb::MocapOdomCallback);
    ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>
        ("odom", 10, fsm_cb::LidarOdomCallback);    
    ros::Subscriber camera_sub = nh.subscribe<nav_msgs::Odometry>
        ("odom", 10, fsm_cb::CameraOdomCallback);
    ros::Subscriber mavros_battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("/mavros/battery", 10, fsm_cb::MavrosBatteryCallback);
    ros::Subscriber mavros_fcu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, fsm_cb::MavrosFcuPoseCallback);
    ros::Subscriber mavros_fcu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, fsm_cb::MavrosFcuVelCallback);
    ros::Subscriber mavros_imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 10, fsm_cb::MavrosImuCallback);
    ros::Subscriber mavros_esc_sub = nh.subscribe<mavros_msgs::ESCStatus>
        ("/mavros/esc/status", 10, fsm_cb::MavrosEscCallback);
    ros::Subscriber mavros_rc_sub = nh.subscribe<mavros_msgs::RCIn>
        ("/mavros/rc/in", 10, fsm_cb::MavrosRcCallback);
    ros::Rate rate(100.0);
    while(ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::PoseStamped mavros_ext_odom_msg;

        if(fsm_cb::is_source_new)
        {
            if(mavros_ext_odom_source == 0) // mocap
            {
                mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::mocap_pos, fsm_cb::mocap_quat);
            }
            else if(mavros_ext_odom_source == 1) // lidar
            {
                mavros_ext_odom_msg = fsm_ut::SetEkfExtPoseData(fsm_cb::lidar_pos, fsm_cb::lidar_quat);
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
            mavros_ext_odom_pub.publish(mavros_ext_odom_msg);
            fsm_cb::is_source_new = false;
        }

        Eigen::Quaterniond mavros_ext_odom_vision_quat = Eigen::Quaterniond(mavros_ext_odom_msg.pose.orientation.w, 
                                                                            mavros_ext_odom_msg.pose.orientation.x, 
                                                                            mavros_ext_odom_msg.pose.orientation.y, 
                                                                            mavros_ext_odom_msg.pose.orientation.z);
        Eigen::Vector3d mavros_ext_odom_vision = fsm_ut::QuatToEuler(mavros_ext_odom_vision_quat);
        if(fabs(mavros_ext_odom_msg.pose.position.x - fsm_cb::mavros_fcu_pos[0]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_msg.pose.position.y - fsm_cb::mavros_fcu_pos[1]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_msg.pose.position.z - fsm_cb::mavros_fcu_pos[2]) < mavros_ekf_pos_error_max && 
           fabs(mavros_ext_odom_vision[2]*180.0/M_PI - fsm_cb::mavros_fcu_euler[2]*180.0/M_PI) < mavros_ekf_yaw_error_max)
        {
            fsm_cb::mavros_ekf_ready = true;
        }
        else 
        {
            fsm_cb::mavros_ekf_ready = false;
        }

        rate.sleep();
    }

    return 0;
}
