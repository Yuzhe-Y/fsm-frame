#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

namespace smacc_ctrl {

  void StLand::onEntry() {
    ROS_INFO("Entering StLand");
  }

  void StLand::onExit() {
    ROS_INFO("Exiting StLand");
  }

}  // namespace smacc_ctrl
