#pragma once

#include <smacc_ctrl/clients/cl_timer/cl_timer.h>

namespace smacc_ctrl
{
    class OrTimer : public smacc::Orthogonal<OrTimer>
    {
        void onInitialize() override
        {
            auto client = this->createClient<ClTimer>();
            client->initialize();
        }
    };
}