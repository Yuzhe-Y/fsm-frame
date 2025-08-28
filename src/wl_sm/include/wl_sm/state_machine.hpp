#pragma once
#include "./state.hpp"
#include "./event.hpp"
#include "ros/subscriber.h"
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "state_msgs/event.h"

namespace wl
{
class StateMachine
{
 private:
  using StateType = State<ros::Subscriber>;
  ros::Subscriber event_sub_;

 public:
  std::vector<StateType> states_;
  std::unordered_map<std::string, int> state_index_map_;

  std::vector<Event> events_;
  std::unordered_map<std::string, int> event_index_map_;

  std::vector<std::vector<int>> transition_table_;

  StateType *current_state_ptr_;
  StateType *next_state_ptr_;

 public:
  StateMachine(std::string json_file)
  {
    using json = nlohmann::json;
    std::ifstream f(json_file);
    json j;
    f >> j;
    // 1. 状态向量 + 索引映射
    int idx = 0;
    for (auto &s : j["states"])
    {
      states_.push_back(StateType(s["name"], idx));
      state_index_map_[s["name"]] = idx++;
    }

    // 2. 事件向量 + 索引映射
    idx = 0;
    for (auto &e : j["events"])
    {
      events_.push_back(Event(e["name"]));
      event_index_map_[e["name"]] = idx++;
    }

    // 3. 创建二维数组，初始化为 -1 表示无效转换
    transition_table_.resize(states_.size(), std::vector<int>(events_.size(), -1));

    // 4. 填充转换表
    for (auto &t : j["transitions"])
    {
      int from_idx = state_index_map_[t["from"]];
      int event_idx = event_index_map_[t["event"]];
      int to_idx = state_index_map_[t["to"]];
      transition_table_[from_idx][event_idx] = to_idx;
    }

    // 5. 初始化当前状态
    current_state_ptr_ = &states_.at(state_index_map_[j["initial_state"]]);
    next_state_ptr_ = current_state_ptr_;
    f.close();

    event_sub_ = ros::NodeHandle().subscribe<state_msgs::event>(
        "state_machine/event", 10, [this](const state_msgs::event::ConstPtr &msg) { this->handle_event(msg->name); });
  }
  StateMachine() = delete;
  StateMachine(const StateMachine &) = delete;
  StateMachine &operator=(const StateMachine &) = delete;

  void init()
  {
    for (auto &s : states_)
    {
      s.init();
    }
  }

  void handle_event(const std::string &event_name)
  {
    if (event_index_map_.find(event_name) == event_index_map_.end())
    {
      std::cout << "Unknown event: " << event_name << std::endl;
      return;
    }

    int e_idx = event_index_map_[event_name];
    std::cout << "handle event: " << event_name << std::endl;
    int next_state_index = transition_table_[current_state_ptr_->id_][e_idx];
    if (next_state_index != -1)
    {
      // std::cout << states_[current_state_] << " --(" << event_name << ")--> " << states_[next_state] << std::endl;
      next_state_ptr_ = &states_.at(next_state_index);
    }
    else
    {
      // std::cout << "No transition from " << states_[current_state_] << " on event " << event_name << std::endl;
    }
  }

  void checkReady()
  {
    // do something
    if (current_state_ptr_ == next_state_ptr_)
    {
      return;
    }
    if (next_state_ptr_->status_ != Status::kReady)
    {
      std::cout << "not ready" << std::endl;
      return;
    }
    std::cout << "ready" << std::endl;
    current_state_ptr_ = next_state_ptr_;
  }

  template <typename... Args>
  void checkReady(std::function<bool(Args...)> condition)
  {
    if (condition == nullptr)
    {
      std::cout << "condition is null" << std::endl;
      return;
    }
    if (condition())
    {
      std::cout << "check if ready" << std::endl;
      current_state_ptr_ = next_state_ptr_;
    }
    else
    {
      std::cout << "not ready" << std::endl;
    }
  }
  auto getCurrentStateName() const { return current_state_ptr_->name_; }
  auto getNextStateName() const { return next_state_ptr_->name_; }
  void printState()
  {
    std::cout << "current state: " << getCurrentStateName() << std::endl;
    std::cout << "next state: " << getNextStateName() << std::endl;
  }
};
}  // namespace wl