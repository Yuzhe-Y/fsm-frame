#include "ros_timer_client/cl_ros_timer.h"
#include "smacc/smacc.h"
#include "smacc_ctrl/public_states.hpp"
#include <smacc_ctrl/clients/cl_state_sub/cl_state_sub.h>
#include <smacc_ctrl/clients/cl_set_mode_srv/cl_set_mode_srv.h>

namespace smacc_ctrl {

    void StArm::onEntry() {
        ROS_INFO("Entering StArm");
    }

    void StArm::time_callback() {
        ClStateSub *cl_state_sub;
        this->getOrthogonal<OrState>()->requiresClient(cl_state_sub);
        auto const *current_state_ptr = cl_state_sub->getState();
        static uint8_t count = 0;
        if (!current_state_ptr->connected) return;

        ClLocalPosPub *cl_local_pos_pub;
        this->getOrthogonal<OrLocalPosPub>()->requiresClient(cl_local_pos_pub);

        ClArmingSrv *cl_arming_srv;
        this->getOrthogonal<OrArming>()->requiresClient(cl_arming_srv);

        ClSetModeSrv *cl_set_mode_srv;
        this->getOrthogonal<OrSetMode>()->requiresClient(cl_set_mode_srv);

        mavros_msgs::SetMode offboard_set_mode;
        offboard_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = 0;
        pose_msg.pose.position.y = 0;
        pose_msg.pose.position.z = 2;

        cl_local_pos_pub->publish(pose_msg);
        static ros::Time last_request = ros::Time::now();
        if (count<100) {
            ++count;
        } else {
            if (ros::Time::now()-last_request > ros::Duration(5.0))
            {
                if (current_state_ptr->mode != "OFFBOARD") {
                    ROS_INFO("Changing to OFFBOARD");
                    if (cl_set_mode_srv->call(offboard_set_mode) &&
                        offboard_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                }
                else if (!current_state_ptr->armed)
                {
                    ROS_INFO("Arming");
                    if (cl_arming_srv->call(arm_cmd) &&
                        arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        if (current_state_ptr->armed && current_state_ptr->mode == "OFFBOARD") {
            this->postEvent<EvArmed>();
        }

    }

    void StArm::onExit() {
        ROS_INFO("Exiting StArm");
        ClTimer *cl_timer;
        this->getOrthogonal<OrTimer>()->requiresClient(cl_timer);
    }

} // namespace smacc_ctrl