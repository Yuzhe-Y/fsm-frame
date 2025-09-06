#pragma once

#include "smacc/smacc.h"
#include <boost/mpl/list.hpp>
#include "./states/forward_states.hpp"
#include "./states/events.hpp"
#include <smacc_ctrl/orthogonals/or_state.h>
#include <smacc_ctrl/clients/cl_local_pos_pub/cl_local_pos_pub.h>
#include <smacc_ctrl/clients/cl_arming_srv/cl_arming_srv.h>
#include <smacc_ctrl/clients/cl_set_mode_srv/cl_set_mode_srv.h>
#include <smacc_ctrl/orthogonals/or_timer.h>
#include <smacc_ctrl/orthogonals/or_state.h>
#include <smacc_ctrl/orthogonals/or_arming.h>
#include <smacc_ctrl/orthogonals/or_set_mode.h>
#include <smacc_ctrl/orthogonals/or_local_pos_pub.h>
#include <smacc_ctrl/orthogonals/or_local_pos_sub.h>
#include <smacc_ctrl/clients/cl_timer/client_behaviors/cb_timer_loop.h>
namespace smacc_ctrl {


class SmExample : public smacc::SmaccStateMachineBase<SmExample, StArm> {
public:
  using SmaccStateMachineBase::SmaccStateMachineBase;
  void onInitialize() override{
    createOrthogonal<OrTimer>();
    createOrthogonal<OrState>();
    createOrthogonal<OrArming>();
    createOrthogonal<OrSetMode>();
    createOrthogonal<OrLocalPosPub>();
    createOrthogonal<OrLocalPosSub>();
  }
};


class StArm : public smacc::SmaccState<StArm, SmExample> {
public:
using SmaccState::SmaccState;
typedef boost::mpl::list<smacc::Transition<EvArmed, StPosCtrl>> reactions;

  void onEntry();

  void onExit();
  void time_callback();

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer,CbTimerLoop>();
  }
  void runtimeConfigure()
  {
    auto cbTimeLoop = getOrthogonal<OrTimer>()->getClientBehavior<CbTimerLoop>();
    cbTimeLoop ->onTimerTick(&StArm::time_callback,this);
  }
};


class StLand : public smacc::SmaccState<StLand, SmExample> {
public:
using SmaccState::SmaccState;
// typedef boost::mpl::list<smacc::Transition<EvArm, StArm>> reactions;

  void onEntry();

  void timerCallback();

  void onExit();

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer,CbTimerLoop>();
  }
  void runtimeConfigure()
  {
    auto cbTimeLoop = getOrthogonal<OrTimer>()->getClientBehavior<CbTimerLoop>();
    cbTimeLoop ->onTimerTick(&StLand::timerCallback,this);
  }
};

class StPosCtrl : public smacc::SmaccState<StPosCtrl, SmExample> {
public:
using SmaccState::SmaccState;
typedef boost::mpl::list<smacc::Transition<EvLand, StLand>> reactions;

  void onEntry();

  void timer_callback();

  void onExit();
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer,CbTimerLoop>();
  }
  void runtimeConfigure()
  {
    auto cbTimeLoop = getOrthogonal<OrTimer>()->getClientBehavior<CbTimerLoop>();
    cbTimeLoop ->onTimerTick(&StPosCtrl::timer_callback,this);
  }
};
} // namespace smacc_ctrl