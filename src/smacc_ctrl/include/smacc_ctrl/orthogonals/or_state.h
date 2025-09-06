#pragma once
#include <smacc_ctrl/clients/cl_state_sub/cl_state_sub.h>

namespace smacc_ctrl
{
    class OrState : public smacc::Orthogonal<OrState>
    {
    public:
        void onInitialize() override
        {
            auto client = createClient<ClStateSub>();
            client -> initialize();
        }
    };
}