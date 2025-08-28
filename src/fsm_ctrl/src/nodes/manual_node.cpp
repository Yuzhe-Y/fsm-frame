#include "ros/ros.h"
#include <wl_sm/state_machine.hpp>

int main(int argc, char **argv)
{

    using namespace wl;
    using State=State<ros::Publisher>;
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;

    State manual_state("manual", Status::kHalt);

    ros::Subscriber sub = nh.subscribe<wl_sm_msgs::state>("state_machine/state", 10, &State::callback, &manual_state);

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