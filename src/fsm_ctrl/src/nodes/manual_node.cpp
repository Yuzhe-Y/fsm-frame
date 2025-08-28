#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>


uint64_t running_time = 0;

void prepare() {
  ROS_INFO("manual preparing");
  running_time = 0;
  sleep(1);
}

void run() {
  ROS_INFO("manual running");
  ++running_time;
  ROS_INFO("running time: %ld s", running_time);
}

void stop() {
  ROS_INFO("manual stopping");
  sleep(1);
}

int main(int argc, char **argv)
{

    using namespace wl;
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;

    State manual_state("manual", prepare, run, stop);

    ros::Rate loop_rate(100);
    manual_state.prepare();

    while (ros::ok())
    {
        manual_state.run();
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    return 0;
}