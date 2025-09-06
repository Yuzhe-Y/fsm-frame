#pragma once
#include <smacc/client_bases/smacc_publisher_client.h>
#include <geometry_msgs/PoseStamped.h>

namespace smacc_ctrl
{
    class ClLocalPosPub : public smacc::client_bases::SmaccPublisherClient
    {
    public:
        ClLocalPosPub(): SmaccPublisherClient()
        {
            this->queueSize = 10;
            this->configure<geometry_msgs::PoseStamped>("mavros/setpoint_position/local");
        }
    };
}