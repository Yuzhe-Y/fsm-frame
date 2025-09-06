#pragma once

#include <moveit/kinematic_constraints/utils.h>

#include "smacc_ctrl/clients/cl_local_pos_pub/cl_local_pos_pub.h"

#include "smacc/smacc.h"

namespace smacc_ctrl
{
    class OrLocalPosPub : public smacc::Orthogonal<OrLocalPosPub>
    {
    public:
        void onInitialize() override
        {
            auto client = createClient<ClLocalPosPub>();
            client->initialize();
        }
    };
}