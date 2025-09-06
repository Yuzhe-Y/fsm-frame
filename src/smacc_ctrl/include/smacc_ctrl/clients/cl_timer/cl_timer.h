#pragma once

#include <ros_timer_client/cl_ros_timer.h>

namespace smacc_ctrl
{
    class ClTimer : public cl_ros_timer::ClRosTimer
    {
    public:
        ClTimer(): cl_ros_timer::ClRosTimer(ros::Duration(0.05))
        {
        }
    };
}