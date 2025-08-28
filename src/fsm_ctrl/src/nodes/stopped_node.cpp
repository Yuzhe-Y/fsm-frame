#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>


int main(int argc, char **argv)
{

    using namespace wl;
    ros::init(argc, argv, "stopped_node");
    ros::NodeHandle nh;

    State Stopped_state("stopped");
    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        if (Stopped_state.status_ == Status::kRunning)
        {
            Stopped_state.run();
        }
        Stopped_state.publishStatus();
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    return 0;
}