#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

namespace smacc_ctrl {

  void StLand::onEntry() {
    ROS_INFO("Entering StLand");
  }

  void StLand::timerCallback()
  {
    ClSetModeSrv *cl_setmode;
    getOrthogonal<OrSetMode>()->requiresClient(cl_setmode);
    mavros_msgs::SetMode set_mode;

    ClStateSub *cl_state_sub;
    getOrthogonal<OrState>()->requiresClient(cl_state_sub);


    static ros::Time last_request = ros::Time::now();
    set_mode.request.custom_mode="AUTO.LAND";

    if (cl_state_sub->getState()->mode!="AUTO.LAND" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (cl_setmode->call(set_mode) &&
          set_mode.response.mode_sent)
      {
        ROS_INFO("Land mode sent");
      }
      last_request = ros::Time::now();
    }
  }
  void StLand::onExit() {
    ROS_INFO("Exiting StLand");
  }

}  // namespace smacc_ctrl
