#pragma once

#include "./status.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "wl_sm_msgs/event.h"
#include "wl_sm_msgs/state.h"
#include "wl_sm_msgs/status.h"
#include <cstring>
#include <iostream>
namespace wl {
class State {
public:
  std::string name_;
  Status status_;
  Status last_status_;
  ros::Publisher pub_status_;
  ros::Publisher pub_ctrl_;
  std::function<void()> prepareFunc_;
  std::function<void()> runFunc_;
  std::function<void()> stopFunc_;

  State(std::string name, std::function<void()> prepareFunc,
        std::function<void()> runFunc, std::function<void()> stopFunc)
      : name_(name), prepareFunc_(prepareFunc), runFunc_(runFunc),
        stopFunc_(stopFunc) {
    status_ = Status::kPreparing;
    last_status_ = Status::kPreparing;
    ros::NodeHandle nh;
    auto pub_name = "state/" + name_;
    pub_status_ = nh.advertise<wl_sm_msgs::status>(pub_name, 10);
    state_sub_ = nh.subscribe<wl_sm_msgs::state>("/state_machine/state", 10,
                                                 &State::callback, this);
  }

  void callback(const wl_sm_msgs::state::ConstPtr &msg) {
    switch (status_) {
    case Status::kReady:
      if (msg->current_name == name_) {
        std::cout << name_ << ": " << "Transitioning to running state" << std::endl;
        status_ = Status::kRunning;
      }
      break;
    case Status::kRunning:
      if (msg->current_name != name_) {
        std::cout << name_ << ": " << "Transitioning to preparing state" << std::endl;
        status_ = Status::kPreparing;
        stop();
        prepare();
      }
      break;
    case Status::kPreparing:
      break;
    }
  }

  // 只能在preparing时调用,结束才进入ready
  void prepare() {
    if (status_ != Status::kPreparing) {
      return;
    }
    if (prepareFunc_ == nullptr) {
      status_ = Status::kReady;
      return;
    }
    prepareFunc_();
    status_ = Status::kReady;
  }

  //只能在running时调用,调用立马进入preparing状态
  void stop() {
    if (status_ != Status::kRunning) {
      return;
    }
    status_ = Status::kPreparing;
    if (stopFunc_ == nullptr) {
      return;
    }
    stopFunc_();
  }

  // 只能在running时调用
  void run(){
    publishStatus();
    if(status_ != Status::kRunning){
      return;
    }
    runFunc_();
  }

  private:
  void publishStatus() {
    if(last_status_ == status_){
      return;
    }
    std::cout <<name_<<":"<< "Publishing status: " << StatusToString(status_) << std::endl;
    wl_sm_msgs::status msg;
    msg.status = StatusToString(status_);
    msg.status_id = static_cast<int>(status_);
    pub_status_.publish(msg);
    last_status_ = status_;
  }


  ros::Subscriber state_sub_;
};
} // namespace wl