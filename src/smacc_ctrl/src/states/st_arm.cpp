#include "ros_timer_client/cl_ros_timer.h"
#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

namespace smacc_ctrl {

    void StArm::onEntry() {
        ROS_INFO("Entering StArm");
        cl_ros_timer::ClRosTimer *cl_timer;
        this->getOrthogonal<OrArm>()->requiresClient(cl_timer);
        cl_timer->onTimerTick(&StArm::time_callback, this);
    }

    void StArm::time_callback() {
        ROS_INFO("Timer ticked");
    }

    void StArm::onExit() {
        ROS_INFO("Exiting StArm");
    }

} // namespace smacc_ctrl