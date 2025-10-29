/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2025-10-29 18
 * @FilePath: /fsm_ctrl/src/nodes/single_offboard_fsm.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by yuzhe-yang, All Rights Reserved. 
 */

/*--------------------------- Write [include] Here ---------------------------*/

#include "fsm_ctrl/utils/fsm_utils/basic_fsm.hpp"

/*--------------------------- Write [variable] Here ---------------------------*/

fsm_ut::FLAG_FSM fsm;

/*--------------------------- Write [function] Here ---------------------------*/



/*--------------------------- Main ---------------------------*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle nh;
    ros::Rate rate(const_params::RATE);

    std::thread udp_thread(&fsm_ut::Basic_FSM::UDPListen, &fsm, 12001);
    udp_thread.detach();

    fsm.Basic_Init(nh);
    
    while (ros::ok())
    {
        ros::spinOnce();
        // if(fsm.real_environment)
        // {}
        fsm.Basic_Task();
        fsm.FLAG_Task();
        rate.sleep();
    }
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