#include "wl_sm/state_machine.hpp"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh;

    wl::StateMachine state_machine("/ws/state_machine.json");

    state_machine.init();

    auto rate = ros::Rate(10);

    while(ros::ok()){
        state_machine.checkReady();
        state_machine.publishState();
        ros::spinOnce();
        rate.sleep();
    }
}