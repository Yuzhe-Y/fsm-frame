#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

namespace smacc_ctrl {

  void StPosCtrl::onEntry() {
    ROS_INFO("Entering StPosCtrl");
  }

  void StPosCtrl::timer_callback()
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 3;
    pose.pose.position.y = 3;
    pose.pose.position.z = 2;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    ClLocalPosPub *local_pos_pub;
    getOrthogonal<OrLocalPosPub>()->requiresClient(local_pos_pub);
    local_pos_pub->publish(pose);

    ClLocalPosSub *local_pos_sub;
    getOrthogonal<OrLocalPosSub>()->requiresClient(local_pos_sub);
    auto current_pose = local_pos_sub->getPose();
    if (abs((pose.pose.position.x-current_pose->pose.position.x)*(pose.pose.position.x-current_pose->pose.position.x)+
            (pose.pose.position.y-current_pose->pose.position.y)*(pose.pose.position.y-current_pose->pose.position.y)) < 0.5)
    {
      this->postEvent<EvLand>();
    }
  }

  void StPosCtrl::onExit() {
    ROS_INFO("Exiting StPosCtrl");
  }

}  // namespace smacc_ctrl
