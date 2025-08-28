#include "ros/ros.h"
#include <bits/stdint-uintn.h>
#include <unistd.h>
#include <wl_sm/state_machine.hpp>

uint64_t running_time = 0;

void prepare() {
  ROS_INFO("automatic preparing");
  running_time = 0;
  sleep(1);
}

void run() {
  ROS_INFO("automatic running");
  ++running_time;
  ROS_INFO("running time: %ld s", running_time);
}

void stop() {
  ROS_INFO("automatic stopping");
  sleep(1);
}
int main(int argc, char **argv) {

  using namespace wl;
  ros::init(argc, argv, "automatic_node");
  ros::NodeHandle nh;

  State automatic_state("automatic", prepare, run, stop);

  ros::Rate loop_rate(100);

  automatic_state.prepare();

  while (ros::ok()) {
    automatic_state.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}