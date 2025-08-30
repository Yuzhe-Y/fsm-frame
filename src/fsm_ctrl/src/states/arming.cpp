#include "wl_sm/state_machine.hpp"
#include "wl_utils/get_mavros.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "state_arming");

  ros::NodeHandle nh;

  wl::MavrosData mavros_data(nh);

  ros::Rate rate(50.0);

  auto prepare = [&rate, &mavros_data]() {
    while (ros::ok() && !mavros_data.current_state_.connected) {
      ros::spinOnce();
      rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
      mavros_data.local_pos_pub_.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
  };

  ros::Time last_request = ros::Time::now();

  auto run = [&rate, &mavros_data, &last_request]() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (mavros_data.current_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (mavros_data.set_mode_client_.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!mavros_data.current_state_.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (mavros_data.arming_client_.call(arm_cmd) &&
            arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_data.local_pos_pub_.publish(pose);
  };

  auto stop = []() { return; };

  wl::State state{"arming", prepare, run, stop};

  state.prepare();
  while (ros::ok()) {
    state.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}