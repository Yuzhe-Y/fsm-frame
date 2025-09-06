#pragma once
#include <smacc_ctrl/clients/cl_local_pos_sub/cl_local_pos_sub.h>
#include <smacc/smacc.h>

namespace smacc_ctrl
{
    class OrLocalPosSub : public smacc::Orthogonal<OrLocalPosSub>
    {
    public:
        void onInitialize() override
        {
            auto client = this->createClient<smacc_ctrl::ClLocalPosSub>();
            client->initialize();
        }
    };
}