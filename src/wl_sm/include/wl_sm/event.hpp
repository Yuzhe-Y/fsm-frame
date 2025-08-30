#pragma once

#include <string>
#include "ros/ros.h"

namespace wl
{
class Event
{
 public:
  std::string name_;
  Event(std::string name) : name_(name) {}
};


}  // namespace wl