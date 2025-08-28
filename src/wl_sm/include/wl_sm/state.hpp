#pragma once

#include "./status.hpp"
#include <cstring>
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "state_msgs/state.h"
#include "state_msgs/status.h"
#include "state_msgs/event.h"
namespace wl
{
template <typename T>
class State;

template <>
class State<ros::Publisher>
{
 public:
  std::string name_;
  Status status_;
  ros::Publisher pub_status_;
  State(std::string name, Status status) : name_(name), status_(status)
  {
    ros::NodeHandle nh;
    auto pub_name = "state/" + name_;
    pub_status_ = nh.advertise<state_msgs::status>(pub_name, 10);
  }

  void callback(const state_msgs::state::ConstPtr& msg)
  {
    switch (status_)
    {
      case Status::kHalt:
        init();
        break;
      case Status::kReady:
        if (msg->current_name == name_)
        {
          status_ = Status::kRunning;
        }
        break;
      case Status::kRunning:
        if (msg->current_name != name_)
        {
          status_ = Status::kPreparing;
          stop();
          prepare();
        }
        break;
      case Status::kPreparing:
        break;
    }
  }

  void init()
  {
    ROS_INFO("State %s: Initing", name_.c_str());
    status_ = Status::kReady;
  }
  void prepare()
  {
    status_ = Status::kPreparing;
    ROS_INFO("State %s: Preparing", name_.c_str());
    status_ = Status::kReady;
    ROS_INFO("State %s: Ready", name_.c_str());
  }
  void run() { ROS_INFO("State %s: Running", name_.c_str()); }
  void stop()
  {
    status_ = Status::kPreparing;
    ROS_INFO("State %s: Stopping", name_.c_str());
  }

  void publishStatus()
  {
    state_msgs::status msg;
    msg.status = StatusToString(status_);
    msg.status_id = static_cast<int>(status_);
    pub_status_.publish(msg);
  }
};

template <>
class State<ros::Subscriber>
{
 public:
  std::string name_;
  int id_;
  Status status_{Status::kHalt};

 private:
  ros::Subscriber sub_status_;

 public:
  State(std::string name, int id) : name_(name), id_(id) {}
  void init()
  {
    ros::NodeHandle nh;
    auto sub_name = "state/" + name_;
    sub_status_ = nh.subscribe<state_msgs::status>(sub_name, 10, &State::callback, this);
  }
  void callback(const state_msgs::status::ConstPtr& msg)
  {
    switch (msg->status_id)
    {
      case -1:
        status_ = Status::kHalt;
        break;
      case 0:
        status_ = Status::kReady;
        break;
      case 1:
        status_ = Status::kRunning;
        break;
      case 2:
        status_ = Status::kPreparing;
        break;
    }
  }
};
}  // namespace wl