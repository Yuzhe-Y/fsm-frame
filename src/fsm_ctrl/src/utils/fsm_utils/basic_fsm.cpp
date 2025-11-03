#include <fsm_ctrl/utils/fsm_utils/basic_fsm.hpp>

namespace fsm_ut
{

Basic_FSM::Basic_FSM(){}

void Basic_FSM::Basic_Init(ros::NodeHandle &nh, ros::Rate rate)                               
{
    /*--------- Normal Publisher ---------*/
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    setpoint_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);
    setpoint_raw_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);

    /*--------- Normal Subscriber ---------*/
    fusion_sub = nh.subscribe<std_msgs::Bool>
        ("/fsm_ctrl/ekf_ready", 1, fsm_cb::MavrosFusionCallback);
    mavros_battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("/mavros/battery", 10, fsm_cb::MavrosBatteryCallback);
    mavros_fcu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, fsm_cb::MavrosFcuPoseCallback);
    mavros_fcu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, fsm_cb::MavrosFcuVelCallback);
    mavros_imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 10, fsm_cb::MavrosImuCallback);
    mavros_esc_sub = nh.subscribe<mavros_msgs::ESCStatus>
        ("/mavros/esc/status", 10, fsm_cb::MavrosEscCallback);
    mavros_rc_sub = nh.subscribe<mavros_msgs::RCIn>
        ("/mavros/rc/in", 10, fsm_cb::MavrosRcCallback);

    /*--------- Timer&&Controller_utils&&Controller Publisher ---------*/
    nh.param("/single_offboard_fsm/controller_basic_params/use_defalut_controller", controller.use_defalut_controller, true);
    nh.param("/single_offboard_fsm/controller_basic_params/defalut_controller_type", controller.defalut_controller_type, 0);
    nh.param("/single_offboard_fsm/controller_basic_params/nmpc_controller_type", controller.nmpc_controller_type, 0);
    nh.param("/single_offboard_fsm/controller_basic_params/hover_thrust_percentage", hover_thrust_percentage, 0.1);
    nh.param("/single_offboard_fsm/controller_basic_params/first_takeoff_height", first_takeoff_height, 0.5);
    nh.param("/single_offboard_fsm/real_environment", real_environment, false);
    fsm.controller_work_enable = false;

    if(controller.use_defalut_controller)
    {
        if(controller.defalut_controller_type == 0) // NMPC(ACADOS)
        {
            if(controller.nmpc_controller_type == 0) // w_and_totalF
            {
                double ctrl_rate, ctrl_duration;
                nh.param("/single_offboard_fsm/acados_parameters/ctrl_rate", ctrl_rate, 50.0);
                ctrl_duration = 1.0 / ctrl_rate;
                fsm_ut::AcadosNmpcWandTotalFControllerInit(nh, acados_simple_controller, ctrl_rate);
                controller_timer = nh.createTimer(ros::Duration(ctrl_duration), fsm_cb::IpoptNmpcWandTotalFTimerCallback);
            }
            else if(controller.nmpc_controller_type == 1) // Force
            {

            }
            else if(controller.nmpc_controller_type == 2) // dForce
            {
                
            }
            else
            {
                ROS_ERROR("Invalid nmpc_controller_type: %d", controller.nmpc_controller_type);
            }
        }
        else if(controller.defalut_controller_type == 1) // NMPC(IPOPT)
        {
            if(controller.nmpc_controller_type == 0) // w_and_totalF
            {
                double ctrl_rate, ctrl_duration;
                nh.param("/single_offboard_fsm/ipopt_parameters/ctrl_rate", ctrl_rate, 50.0);
                ctrl_duration = 1.0 / ctrl_rate;
                fsm_ut::IpoptNmpcWandTotalFControllerInit(nh, nmpc_controller_w_and_totalF);
                controller_timer = nh.createTimer(ros::Duration(ctrl_duration), fsm_cb::IpoptNmpcWandTotalFTimerCallback);
                // nmpc_state_pub = nh.advertise<fsm_ctrl::nmpc_simple_model_msgs>("/fsm_ctrl/nmpc_state", 10);
            }
            else if(controller.nmpc_controller_type == 1) // Force
            {

            }
            else if(controller.nmpc_controller_type == 2) // dForce
            {
                
            }
            else
            {
                ROS_ERROR("Invalid nmpc_controller_type: %d", controller.nmpc_controller_type);
            }
        }
        else if(controller.defalut_controller_type == 2) // attitude
        {
            double ctrl_rate, ctrl_duration;
            nh.param("/single_offboard_fsm/dfbc_parameters/ctrl_rate", ctrl_rate, 50.0);
            ctrl_duration = 1.0 / ctrl_rate;
            // fsm_ut::DFBCControllerInit(nh, dfbc_controller);
            // controller_timer = nh.createTimer(ros::Duration(ctrl_duration), fsm_cb::DFBCTimerCallback);
        }
        else
        {
            ROS_ERROR("Invalid defalut_controller_type: %d", controller.defalut_controller_type);
        }
    }
    else
    {
        ROS_WARN("Not using defalut controller!");
    }
    
    is_landing_in_progress = false;

    /*--------- Client ---------*/
    arming_cmd_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    last_request = ros::Time::now().toSec();

    if(real_environment)
    {fsm_ut::InitPX4(offboard_mode, land_mode, arm_cmd, disarm_cmd, setpoint_pos_pub, rate);}
}


FLAG_FSM::FLAG_FSM(): Basic_FSM() 
{
    std::cout << "*************************" << std::endl;
    std::cout << "* Make FLAG Great Again *" << std::endl;
    std::cout << "*************************" << std::endl;
}


void Basic_FSM::UDPListen(const uint16_t cport) 
{   
    /* create socket */
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);    // AF_INET: IPv4    
    if(sock_fd < 0)                                  // SOCK_DGRAM: UDP
    {                                                // 0: default protocol
        ROS_ERROR("Fail to Create Socket!!!");            
        return;                                           
    }

    /* describe IPv4 port */
    sockaddr_in addr_lis;
    memset(&addr_lis, 0, sizeof(sockaddr_in));       // initialize storage addresses to 0
    addr_lis.sin_family = AF_INET;                   // IPv4
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY);    // set address automatically, converted from host to network byte order
    addr_lis.sin_port = htons(cport);                // set port number, converted from host to network byte order

    /* bind socket with port */
    if(bind(sock_fd, (sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
        ROS_ERROR("Fail to Bind Socket with Port!!!");
        return;
    }

    int recv_num;                  // number of bytes received
    char recv_buf[100];            // buffer for received data
    sockaddr_in addr_client;       // storage of client address port
    int len = sizeof(addr_lis);    // length of address port
    const char dot[2] = ",";       // ", \0"


    while (ros::ok())    // `recvfrom()` block until data is received
    {
        char *str;
        int user_cmd;
        double init_px, init_py, init_pz;

        /* receive UDP data */
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num < 0 || abs(recv_num - 19) > 3)
        {
            ROS_ERROR("Fail to Receive UDP data!!!");
            continue;
        }
        recv_buf[recv_num] = '\0';    // add string terminator
        // ROS_INFO("Rec: %s, len = %d", recv_buf, recv_num);
        str = strtok(recv_buf, dot);
        sscanf(str, "%d", &user_cmd);
        str = strtok(NULL, dot);
        sscanf(str, "%lf", &init_px);
        str = strtok(NULL, dot);
        sscanf(str, "%lf", &init_py);
        str = strtok(NULL, dot);         // separate string
        sscanf(str, "%lf", &init_pz);    // convert string to int | double

        if(cmd < -1 || cmd > 9) {ROS_ERROR("Invalid Command!!!");}

        /* remind EKF convergence */
        if(cmd == -1)
        {
            if(is_ekf_converge) {ROS_WARN("Localization Ready!");}
            else                {ROS_ERROR("EKF Not Convergent!!!");}
        }
        
        /* safty protection land and disable UDP */
        if(is_safety_trigger)
        {
            cmd = 5;
            is_udp_enable = false;
            ROS_WARN("Safty Protection Land!");
        }

        /* accept user command */
        if(is_udp_enable) {cmd = user_cmd;}
    }
}



void Basic_FSM::Basic_Task()
{   
    last_cmd = cmd; //命令保存
    last_cmd_time = now_cmd_time;
    now_cmd_time = ros::Time::now().toSec() - last_request;
    /*--------- Arm ---------*/
    if(cmd == 1)
    {
        // if(mavros_mode != "OFFBOARD")
        // { 
        //     if(ros::Time::now() - mavros_state_monitor_time > ros::Duration(3.0))
        //     {
        //         ROS_INFO("Attempt to Set OFFBOARD");
        //         if(set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent) {ROS_WARN("Mode Offboard!");}
        //         else {ROS_ERROR("Fail to Set OFFBOARD!!!");}
        //         mavros_state_monitor_time = ros::Time::now();
        //     }                
        // }
        // else if(mavros_mode == "OFFBOARD" && !arm_mode)
        // {
        //     if(ros::Time::now() - mavros_state_monitor_time > ros::Duration(3.0))
        //     {
        //         ROS_INFO("Attempt to Arm");
        //         if(arming_cmd_client.call(arm_cmd) && arm_cmd.response.success) {ROS_WARN("Mode Armed!");}
        //         else {ROS_ERROR("Fail to Arm!!!");}
        //         mavros_state_monitor_time = ros::Time::now();
        //     }
        // }
        // else {ROS_WARN("FLAG Fly!");}

        // mavros_msgs::AttitudeTarget att_cmd = fsm_ut::SetTargetRateAndTotalThrustCmd(0.0, 0.0, 0.0, 0.02);
        // setpoint_raw_att_pub.publish(att_cmd);
    }


    /*--------- Hover ---------*/
    else if(cmd == 2)
    {
        geometry_msgs::PoseStamped pos_cmd = fsm_ut::SetPositionAndYawCmd(0.0, 0.0, first_takeoff_height, 0.0);
        setpoint_pos_pub.publish(pos_cmd);
    }


    /*--------- Takeoff ---------*/
    else if(cmd == 3)
    {
        // if(mavros_mode != "OFFBOARD")
        // { 
        //     if(ros::Time::now() - mavros_state_monitor_time > ros::Duration(3.0))
        //     {
        //         ROS_INFO("Attempt to Set OFFBOARD");
        //         if(set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent) {ROS_WARN("Mode Offboard!");}
        //         else {ROS_ERROR("Fail to Set OFFBOARD!!!");}
        //         mavros_state_monitor_time = ros::Time::now();
        //     }                
        // }
        // else if(mavros_mode == "OFFBOARD" && !arm_mode)
        // {
        //     if(ros::Time::now() - mavros_state_monitor_time > ros::Duration(3.0))
        //     {
        //         ROS_INFO("Attempt to Arm");
        //         if(arming_cmd_client.call(arm_cmd) && arm_cmd.response.success) {ROS_WARN("Mode Armed!");}
        //         else {ROS_ERROR("Fail to Arm!!!");}
        //         mavros_state_monitor_time = ros::Time::now();
        //     }
        // }
        // else {ROS_WARN("FLAG Fly!");}

        // geometry_msgs::PoseStamped pos_cmd = fsm_ut::SetPositionAndYawCmd(0.0, 0.0, first_takeoff_height, 0.0);
        // setpoint_pos_pub.publish(pos_cmd);
    }


    /*--------- Land ---------*/
    else if(cmd == 4)
    {
        // if(arm_mode && abs(fsm_cb::mavros_fcu_pos.z() - 0.05) < 0.05)
        // {
        //     if(ros::Time::now() - mavros_state_monitor_time > ros::Duration(3.0))
        //     {
        //         ROS_INFO("Attempt to Disarm");
        //         if(arming_cmd_client.call(disarm_cmd) && disarm_cmd.response.success) {ROS_WARN("Mode Disarm!");}
        //         else {ROS_ERROR("Fail to Disarm!!!");}
        //         mavros_state_monitor_time = ros::Time::now();
        //     }
        // }
        
        // if(!is_landing_in_progress)
        // {
        //     landing_start_pos = Eigen::Vector3d(fsm_cb::mavros_fcu_pos.x(), fsm_cb::mavros_fcu_pos.y(), fsm_cb::mavros_fcu_pos.z());
        //     is_landing_in_progress = true;
        // }
        // if(landing_start_pos.z() > 0.05) {landing_start_pos.z() -= 0.01;}
        // else {landing_start_pos.z() = 0.05;}

        // geometry_msgs::PoseStamped pos_cmd = fsm_ut::SetPositionAndYawCmd(landing_start_pos.x(), landing_start_pos.y(), landing_start_pos.z(), 0.0);
        // setpoint_pos_pub.publish(pos_cmd);
    }


    /*--------- Safety ---------*/
    else if(cmd == 5)
    {
        mavros_msgs::AttitudeTarget att_cmd = fsm_ut::SetTargetRateAndTotalThrustCmd(0.0, 0.0, 0.0, hover_thrust_percentage - 0.05);
        setpoint_raw_att_pub.publish(att_cmd);
    }

    else {return;}
}

} //namespace fsm_ut