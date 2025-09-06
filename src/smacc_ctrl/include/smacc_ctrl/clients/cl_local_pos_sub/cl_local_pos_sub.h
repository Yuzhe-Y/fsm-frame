#pragma once

#include "smacc/client_bases/smacc_subscriber_client.h"
#include "geometry_msgs/PoseStamped.h"

namespace smacc_ctrl
{
    class ClLocalPosSub : public smacc::client_bases::SmaccSubscriberClient<geometry_msgs::PoseStamped>
    {
    private:
        using TSubscriberBase = smacc::client_bases::SmaccSubscriberClient<geometry_msgs::PoseStamped>;
        geometry_msgs::PoseStamped pose_;

        void onPoseReceived(const geometry_msgs::PoseStamped &msg)
        {
            pose_ = msg;
        }
    public:
        ClLocalPosSub() : SmaccSubscriberClient("/mavros/local_position/pose")
        {
        }
        void initialize() override
        {
            TSubscriberBase::initialize();
            onMessageReceived(&ClLocalPosSub::onPoseReceived, this);
        }
        const geometry_msgs::PoseStamped *getPose() const
        {
            return &pose_;
        }
    };
}