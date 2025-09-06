#pragma once
#include <mavros_msgs/CommandBool.h>
#include <smacc/client_bases/smacc_service_client.h>

namespace smacc_ctrl
{
    class ClArmingSrv : public smacc::client_bases::SmaccServiceClient<mavros_msgs::CommandBool>
    {
    public:
        ClArmingSrv(): smacc::client_bases::SmaccServiceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")
        {
        }
    };
}