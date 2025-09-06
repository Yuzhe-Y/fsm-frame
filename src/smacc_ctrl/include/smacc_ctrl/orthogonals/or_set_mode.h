#pragma once

#include <smacc_ctrl/clients/cl_set_mode_srv/cl_set_mode_srv.h>

#include "smacc/impl/smacc_orthogonal_impl.h"
#include "smacc/smacc.h"

namespace smacc_ctrl
{
    class OrSetMode : public smacc::Orthogonal<OrSetMode>
    {
    public:
        void onInitialize() override
        {
            auto client = createClient<ClSetModeSrv>();
            client->initialize();
        }
    };
}
