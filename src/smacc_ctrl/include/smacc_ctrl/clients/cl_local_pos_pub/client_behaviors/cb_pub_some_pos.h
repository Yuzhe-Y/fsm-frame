#pragma once
#include <smacc/smacc.h>
#include <smacc_ctrl/clients/cl_local_pos_pub/cl_local_pos_pub.h>


namespace smacc_ctrl
{
    class CbPubSomePos : public smacc::SmaccClientBehavior
    {
    public:
        CbPubSomePos(unsigned int counter = 10, ros::Duration period = ros::Duration(1.0))
            : counter_(counter), duration_(period)
        {
        }
        void onEntry() override
        {
            requiresClient(localPosPubClient_);

        }



    private:
        ClLocalPosPub *localPosPubClient_;
        unsigned int counter_;
        ros::Duration duration_;
    };
}