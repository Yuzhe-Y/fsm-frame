#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>

int main(int argc, char **argv)
{

    using namespace wl;
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;

    State manual_state("manual");

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        if (manual_state.status_ == Status::kRunning)
        {
            manual_state.run();
        }
        manual_state.publishStatus();
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    return 0;
}