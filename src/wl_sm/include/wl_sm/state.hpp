#pragma once

#include "./status.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "wl_sm_msgs/event.h"
#include "wl_sm_msgs/state.h"
#include "wl_sm_msgs/status.h"
#include <cstring>
namespace wl {
class State {
public:
  std::string name_;
  Status status_;
  ros::Publisher pub_status_;
  State(std::string name) : name_(name) {
    status_ = Status::kHalt;
    ros::NodeHandle nh;
    auto pub_name = "state/" + name_;
    pub_status_ = nh.advertise<wl_sm_msgs::status>(pub_name, 10);

    state_sub_ = nh.subscribe<wl_sm_msgs::state>("/state_machine/state", 10,
                                                &State::callback, this);
  }

  State(std::string name, Status status) : name_(name), status_(status) {
    ros::NodeHandle nh;
    auto pub_name = "state/" + name_;
    pub_status_ = nh.advertise<wl_sm_msgs::status>(pub_name, 10);

    state_sub_ = nh.subscribe<wl_sm_msgs::state>("/state_machine/state", 10,
                                                &State::callback, this);
  }

  void callback(const wl_sm_msgs::state::ConstPtr &msg) {
    switch (status_) {
    case Status::kHalt:
      init();
      break;
    case Status::kReady:
      if (msg->current_name == name_) {
        status_ = Status::kRunning;
      }
      break;
    case Status::kRunning:
      if (msg->current_name != name_) {
        status_ = Status::kPreparing;
        stop();
        prepare();
      }
      break;
    case Status::kPreparing:
      break;
    }
  }

  void init() {
    ROS_INFO("State %s: Initing", name_.c_str());
    status_ = Status::kReady;
  }
  void prepare() {
    status_ = Status::kPreparing;
    ROS_INFO("State %s: Preparing", name_.c_str());
    status_ = Status::kReady;
    ROS_INFO("State %s: Ready", name_.c_str());
  }
  void run() { ROS_INFO("State %s: Running", name_.c_str()); }
  void stop() {
    status_ = Status::kPreparing;
    ROS_INFO("State %s: Stopping", name_.c_str());
  }

  void publishStatus() {
    wl_sm_msgs::status msg;
    msg.status = StatusToString(status_);
    msg.status_id = static_cast<int>(status_);
    pub_status_.publish(msg);
  }

private:
  ros::Subscriber state_sub_;
};
} // namespace wl