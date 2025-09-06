#pragma once

#include <smacc/client_bases/smacc_service_client.h>
#include <mavros_msgs/SetMode.h>

namespace smacc_ctrl
{
    class ClSetModeSrv : public smacc::client_bases::SmaccServiceClient<mavros_msgs::SetMode>
    {
    public:
        ClSetModeSrv() : smacc::client_bases::SmaccServiceClient<mavros_msgs::SetMode>("mavros/set_mode")
        {
        }
    };
}