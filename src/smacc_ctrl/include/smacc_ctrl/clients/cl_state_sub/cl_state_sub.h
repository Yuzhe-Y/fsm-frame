#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <mavros_msgs/State.h>

namespace smacc_ctrl
{
    class ClStateSub : public smacc::client_bases::SmaccSubscriberClient<mavros_msgs::State>
    {
    private:
        using TSubscriberClient = smacc::client_bases::SmaccSubscriberClient<mavros_msgs::State>;
        mavros_msgs::State current_state;
        mavros_msgs::State* const current_state_ptr = &current_state;
    public:
        ClStateSub(): smacc::client_bases::SmaccSubscriberClient<mavros_msgs::State>("mavros/state")
        {
            this->queueSize=10;
        }
        void initialize() override
        {
            TSubscriberClient::initialize();
            onMessageReceived(&ClStateSub::updateState,this);
        }
        auto getState() const
        {
            return current_state_ptr;
        }
    private:
        void updateState(const mavros_msgs::State& msg)
        {
            this->current_state = msg;
        }
    };
}