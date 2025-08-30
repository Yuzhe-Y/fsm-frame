#include "ros/ros.h"
#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "smacc_ctrl_state_machine");
  ros::NodeHandle nh;

  ros::Rate rate(10);

  smacc::run<smacc_ctrl::SmExample>();

  static int count = 0;

  while (ros::ok()) {
    if(count > 100){
      // postEvent<smacc_ctrl::EvPosCtrl>();
    }else {
      count++;
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}