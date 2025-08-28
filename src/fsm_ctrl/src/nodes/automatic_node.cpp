#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>

int main(int argc, char **argv)
{

    using namespace wl;
    ros::init(argc, argv, "automatic_node");
    ros::NodeHandle nh;

    State automatic_state("automatic");

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        if (automatic_state.status_ == Status::kRunning)
        {
            automatic_state.run();
        }
        automatic_state.publishStatus();
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    return 0;
}