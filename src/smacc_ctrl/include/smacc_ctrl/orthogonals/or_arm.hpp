#pragma once
#include "smacc/smacc.h"
#include "smacc/smacc.h"
#include "ros_timer_client/cl_ros_timer.h"
namespace smacc_ctrl {
    struct OrArm : smacc::Orthogonal<OrArm> {
        void onInitialize() override {
        auto timeClient = this->createClient<cl_ros_timer::ClRosTimer>(ros::Duration(0.1));
        timeClient->initialize();
    } // onInitialize
    };
}