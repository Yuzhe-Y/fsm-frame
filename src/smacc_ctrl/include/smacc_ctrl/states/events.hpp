#pragma once
#include "./forward_states.hpp"

namespace smacc_ctrl {
struct EvArm : sc::event<EvArm> {};
    struct EvArmed : public sc::event<EvArmed>{};
struct EvPosCtrl : sc::event<EvPosCtrl> {};
struct EvLand : sc::event<EvLand> {};
}