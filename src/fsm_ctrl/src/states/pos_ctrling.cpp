#include "wl_sm/state_machine.hpp"
#include "wl_sm_msgs/event.h"
#include "wl_utils/get_mavros.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_pos_ctrling");

  ros::NodeHandle nh;

  wl::MavrosData mavros_data(nh);

  ros::Rate rate(20.0);

  auto prepare=[&rate, &mavros_data](){
    while (ros::ok() && !mavros_data.current_state_.connected) {
      ros::spinOnce();
      rate.sleep();
    }
  };
  auto run=[&rate, &mavros_data,&nh](){
    static auto pub = nh.advertise<wl_sm_msgs::event>("state_machine/event", 10);
    if(!mavros_data.current_state_.armed){
        wl_sm_msgs::event e;
        e.name = "not_armed";
        pub.publish(e);
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 10;
    pose.pose.position.y = 10;
    pose.pose.position.z = 10;

    mavros_data.local_pos_pub_.publish(pose);
  };
  auto stop=[](){
    return;
  };

  wl::State state{"pos_ctrling", prepare, run, stop};

  state.prepare();
  while(ros::ok()){
    state.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}