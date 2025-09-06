#pragma once
#include <smacc/smacc.h>
#include <smacc_ctrl/clients/cl_timer/cl_timer.h>

namespace smacc_ctrl
{
    class CbTimerLoop : public smacc::SmaccClientBehavior
    {
    public:
        void onEntry() override
        {
            this->requiresClient(timerClient_);

            timerClient_->onTimerTick(&CbTimerLoop::onClientTimerTickCallback, this);
        }
        void onExit() override
        {

        }

        template <typename TOrthogonal, typename TSourceObject>
        void onOrthogonalAllocation()
        {
            this->postTimerEvent_ = [=]() {
                // this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>();
            };
        }

        void onClientTimerTickCallback()
        {
            this->postTimerEvent_();
            onTimerTick_();
        }

        template <typename T>
        boost::signals2::connection onTimerTick(void (T::*callback)(), T *object)
        {
            return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
        }

    private:
        ClTimer *timerClient_;
        std::function<void()> postTimerEvent_;
        smacc::SmaccSignal<void()> onTimerTick_;
    };
}
