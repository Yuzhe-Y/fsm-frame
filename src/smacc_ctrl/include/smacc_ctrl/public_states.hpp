#pragma once

#include "smacc/smacc.h"
#include <boost/mpl/list.hpp>
#include "./states/forward_states.hpp"
#include "./states/events.hpp"
#include "./orthogonals/or_arm.hpp"

namespace smacc_ctrl {


class SmExample : public smacc::SmaccStateMachineBase<SmExample, StArm> {
public:
  using SmaccStateMachineBase::SmaccStateMachineBase;
  void onInitialize() override{
    // Initialize the orthogonal
    this->createOrthogonal<OrArm>();
  }
};


class StArm : public smacc::SmaccState<StArm, SmExample> {
public:
using SmaccState::SmaccState;
typedef boost::mpl::list<smacc::Transition<EvPosCtrl, StPosCtrl>> reactions;

  void onEntry();

  void onExit();
  void time_callback();
};


class StLand : public smacc::SmaccState<StLand, SmExample> {
public:
using SmaccState::SmaccState;
typedef boost::mpl::list<smacc::Transition<EvArm, StArm>> reactions;

  void onEntry();

  void onExit();
};

class StPosCtrl : public smacc::SmaccState<StPosCtrl, SmExample> {
public:
using SmaccState::SmaccState;
typedef boost::mpl::list<smacc::Transition<EvLand, StLand>> reactions;

  void onEntry();

  void onExit();
};
} // namespace smacc_ctrl