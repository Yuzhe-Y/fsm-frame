#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

namespace wl {

struct MavrosData {
  mavros_msgs::State current_state_;
  ros::Subscriber state_sub_;
  ros::Publisher local_pos_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  MavrosData(ros::NodeHandle &nh) {
    state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10,
                                                  &MavrosData::state_cb, this);
    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    arming_client_ =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  }
  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
  }
};

} // namespace wl