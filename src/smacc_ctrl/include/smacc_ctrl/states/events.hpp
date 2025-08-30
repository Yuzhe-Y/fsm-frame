#pragma once
#include "./forward_states.hpp"

namespace smacc_ctrl {
struct EvArm : sc::event<EvArm> {};
struct EvPosCtrl : sc::event<EvPosCtrl> {};
struct EvLand : sc::event<EvLand> {};
}