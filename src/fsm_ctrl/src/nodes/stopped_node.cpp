#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>

uint64_t running_time = 0;

void prepare() {
  ROS_INFO("stopped preparing");
  running_time = 0;
  sleep(1);
}

void run() {
  ROS_INFO("stopped running");
  ++running_time;
  ROS_INFO("running time: %ld s", running_time);
}

void stop() {
  ROS_INFO("stopped stopping");
  sleep(1);
}

int main(int argc, char **argv) {

  using namespace wl;
  ros::init(argc, argv, "stopped_node");
  ros::NodeHandle nh;

  State stopped_state("stopped", prepare, run, stop);
  ros::Rate loop_rate(100);
  stopped_state.prepare();
  while (ros::ok()) {
    stopped_state.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}