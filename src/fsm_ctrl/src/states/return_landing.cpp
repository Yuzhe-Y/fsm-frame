#include "ros/time.h"
#include "wl_sm/state_machine.hpp"
#include "wl_utils/get_mavros.hpp"
#include "cmath"


int main(int argc, char** argv){
  ros::init(argc, argv, "state_return_landing");

  ros::NodeHandle nh;

  wl::MavrosData mavros_data(nh);

  ros::Rate rate(50.0);

  auto prepare=[&rate, &mavros_data](){
    while (ros::ok() && !mavros_data.current_state_.connected) {
      ros::spinOnce();
      rate.sleep();
    }
  };
  auto run=[&rate, &mavros_data](){
    auto last_request = ros::Time::now();
    if(abs(mavros_data.current_state_.mode.compare("AUTO.LAND"))>0){
      mavros_msgs::SetMode land_set_mode;
      land_set_mode.request.custom_mode = "AUTO.LAND";
      if (mavros_data.set_mode_client_.call(land_set_mode) &&
          land_set_mode.response.mode_sent &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
        ROS_INFO("Landing enabled");
        last_request = ros::Time::now();
      }
    }
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