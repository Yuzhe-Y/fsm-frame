#pragma once
#include <smacc_ctrl/clients/cl_arming_srv/cl_arming_srv.h>
#include <smacc/smacc.h>

namespace smacc_ctrl
{
    class OrArming : public smacc::Orthogonal<OrArming>
    {
    public:
        void onInitialize() override
        {
            auto client = createClient<ClArmingSrv>();
            client->initialize();
        }
    };
}