#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

namespace smacc_ctrl {

  void StPosCtrl::onEntry() {
    ROS_INFO("Entering StPosCtrl");
  }

  void StPosCtrl::onExit() {
    ROS_INFO("Exiting StPosCtrl");
  }

}  // namespace smacc_ctrl
