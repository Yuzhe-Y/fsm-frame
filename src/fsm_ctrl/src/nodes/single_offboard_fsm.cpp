/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-11-03 11
 * @FilePath: /fsm_ctrl/src/nodes/single_offboard_fsm.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */

/*--------------------------- Write [include] Here ---------------------------*/

#include "fsm_ctrl/utils/fsm_utils/basic_fsm.hpp"
#include <signal.h>

/*--------------------------- Write [variable] Here ---------------------------*/

// 全局 FSM 实例
fsm_ut::FLAG_FSM fsm;

/*--------------------------- Write [function] Here ---------------------------*/

// 在 Ctrl+C（SIGINT）时调用的处理函数，确保优雅关闭
static void SigIntHandler(int /*sig*/)
{
    // 优雅关闭 FSM（关闭线程、socket、timer 等）
    fsm.Shutdown();

    // 告知 ROS 退出
    ros::shutdown();

    // 直接返回，让程序按正常流程退出
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle nh;

    // 在调用 Basic_Init 之前先构造 rate，因为 Basic_Init 需要传入 rate
    ros::Rate rate(const_params::RATE);

    // 注册信号处理器，用于 Ctrl+C 时优雅退出
    signal(SIGINT, SigIntHandler);

    // 初始化 FSM（传入 nh 和 rate）
    fsm.Basic_Init(nh, rate);

    // 启动 UDP 监听（不再 detach）
    fsm.StartUDPListen(12001);

    // 主循环
    while (ros::ok())
    {
        ros::spinOnce();
        // if(fsm.real_environment)
        // {}
        fsm.Basic_Task();
        fsm.FLAG_Task();
        // std::cout << "mavros_fcu_pos:" << fsm_cb::mavros_fcu_pos.transpose() <<std::endl;
        // std::cout << "mavros_fcu_vel:" << fsm_cb::mavros_fcu_vel.transpose() <<std::endl;
        // std::cout << "mavros_imu_rate:" << fsm_cb::mavros_imu_rate.transpose() <<std::endl;
        rate.sleep();
    }

    // 程序退出前再次确保 Shutdown 已调用（安全）
    fsm.Shutdown();
    return 0;
}

/*--------------------------- Write [task] Here ---------------------------*/

void fsm_ut::FLAG_FSM::FLAG_Task()
{
    if(cmd == 6)
    {  
        
    }
    

    else if(cmd == 7)
    {
        
    }
    
    
    else if(cmd == 8)
    {
        
    }
    
    
    else if(cmd == 9)
    {

    }

    else {return;}
}