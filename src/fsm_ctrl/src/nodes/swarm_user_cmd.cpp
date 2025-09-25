/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-09-24 22
 * @FilePath: /fsm_ctrl/src/nodes/swarm_user_cmd.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

#include "fsm_ctrl/callbacks/external_odom_callback.hpp"
#include "fsm_ctrl/callbacks/mavros_callback.hpp"
#include "fsm_ctrl/utils/math_utils/math_utils.hpp"

using namespace std;

static int cmd = 0;

Eigen::Vector3d mavros_ext_odom_pos; // 外部里程计位置
Eigen::Quaterniond mavros_ext_odom_quat; // 外部里程计四元数
double mavros_ext_odom_yaw = 0.0; // 外部里程计偏航角

void CMD_Print()
{
    cout.setf(ios::fixed);             // 固定的浮点显示
    cout << setprecision(3);           // 固定显示精度为2位
    cout.setf(ios::left);              // 左对齐
    cout.setf(ios::showpoint);         // 强制显示小数点
    cout.setf(ios::showpos);           // 强制显示符号
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>--- User Info ---<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "1: Arm          2: Takeoff  3: Point_target_fly  4: Space_explore  5: hover" << endl;
    cout << "6: Land&Disarm  7:          8: Auto              9:                10: Exit" << endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>--- Pose Info ---<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "[FCU]  x: " << fsm_cb::mavros_fcu_pos(0) << "(m)    y: " << fsm_cb::mavros_fcu_pos(1) << "(m)    z: " ;
    cout <<                 fsm_cb::mavros_fcu_pos(2) << "(m)    yaw: " << fsm_cb::mavros_fcu_euler(2) << "(deg)" << endl;
    cout << "[EXT]  x: " << mavros_ext_odom_pos(0) << "(m)    y: " << mavros_ext_odom_pos(1) << "(m)    z: " ;
    cout <<                 mavros_ext_odom_pos(2) << "(m)    yaw: " << mavros_ext_odom_yaw << "(deg)" << endl;
    cout.unsetf(ios::showpos);
    cout << "[CMD]  " << cmd << endl;
    cout << endl;
}


void UdpServer(const char* ip, const uint16_t cport, const int UAVID)
{
    ROS_INFO("UDP %d", UAVID);
    string ss;
    geometry_msgs::PoseStamped offset;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }
    struct sockaddr_in addr_client;
    int len;
    memset(&addr_client, 0, sizeof(struct sockaddr_in));
    addr_client.sin_family = AF_INET;
    addr_client.sin_addr.s_addr = inet_addr(ip);
    addr_client.sin_port = htons(cport);
    len = sizeof(addr_client);

    offset.pose.position.x = 0.0;
    offset.pose.position.y = 0.0;
    offset.pose.position.z = 0.0;

    int sdlen;
    int send_num;
    char send_buf[100];

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        memset(&send_buf, 0, sizeof(send_buf));
        sdlen = 0;

        sprintf(send_buf + sdlen, "%d", cmd >= 10? 10:cmd);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.x);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.y);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.z);
        
        send_num = sendto(sock_fd, send_buf, ssize_t(strlen(send_buf)), 0, (struct sockaddr*)&addr_client, len);
        if(send_num < 0)
        {
            ROS_ERROR("Send Fail!, UAV = %d",UAVID);
            // perror("sendto error:");
            // exit(1);
        }
        // ROS_INFO("Current Pub: %s", send_buf);
        // cout << ssize_t(strlen(send_buf)) << endl;
        if(cmd == 10) {break;}
        rate.sleep();
    }
}


void UserCmd()
{
    cin >> cmd;
}

void CmdListener()
{
    ros::Rate rate(10.0);
    while(ros::ok())
    {
        UserCmd();
        if(cmd == 10) {break;}
        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_user_cmd");
    ros::NodeHandle nh;

    int mavros_ext_odom_source = 0;

    nh.param("mavros_ext_odom_source", mavros_ext_odom_source, 0);
    
    new thread(&CmdListener);
    new thread(&UdpServer, "127.0.0.1", 12001, 1);

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        if(mavros_ext_odom_source == 0) // mocap
        {
            mavros_ext_odom_pos = fsm_cb::mocap_pos;
            mavros_ext_odom_quat = fsm_cb::mocap_quat;
            mavros_ext_odom_yaw = fsm_ut::QuatToEuler(mavros_ext_odom_quat).z();
        }
        else if(mavros_ext_odom_source == 1) // lidar
        {
            mavros_ext_odom_pos = fsm_cb::lidar_pos;
            mavros_ext_odom_quat = fsm_cb::lidar_quat;
            mavros_ext_odom_yaw = fsm_ut::QuatToEuler(mavros_ext_odom_quat).z();
        }
        else if(mavros_ext_odom_source == 2) // camera
        {
            mavros_ext_odom_pos = fsm_cb::camera_pos;
            mavros_ext_odom_quat = fsm_cb::camera_quat;
            mavros_ext_odom_yaw = fsm_ut::QuatToEuler(mavros_ext_odom_quat).z();
        }
        else
        {
            ROS_ERROR("Invalid mavros_ext_odom_source: %d", mavros_ext_odom_source);
            continue;
        }

        CMD_Print();
        if(cmd == 10) {break;}
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
