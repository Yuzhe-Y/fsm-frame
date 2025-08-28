#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>

int main(int argc, char **argv)
{

    using namespace wl;
    using State=State<ros::Publisher>;
    ros::init(argc, argv, "automatic_node");
    ros::NodeHandle nh;

    State automatic_state("automatic", Status::kHalt);

    ros::Subscriber sub = nh.subscribe<wl_sm_msgs::state>("state_machine/state", 10, &State::callback, &automatic_state);

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