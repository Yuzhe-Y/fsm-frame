#include <fsm_ctrl/utils/fsm_utils/basic_fsm.hpp>

namespace fsm_ut
{

Basic_FSM::Basic_FSM(){}

void Basic_FSM::Basic_Init(ros::NodeHandle &nh)                               
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

    /*--------- Timer&&Controller_utils&&Controller Publisher ---------*/
    nh.param("/single_offboard_fsm/controller_basic_params/use_defalut_controller", controller.use_defalut_controller, true);
    nh.param("/single_offboard_fsm/controller_basic_params/defalut_controller_type", controller.defalut_controller_type, 0);
    nh.param("/single_offboard_fsm/controller_basic_params/nmpc_controller_type", controller.nmpc_controller_type, 0);
    nh.param("/single_offboard_fsm/controller_basic_params/hover_thrust_percentage", hover_thrust_percentage, 0.1);
    nh.param("/single_offboard_fsm/controller_basic_params/first_takeoff_height", first_takeoff_height, 0.5);
    fsm.controller_work_enable = false;

    if(controller.use_defalut_controller)
    {
        if(controller.defalut_controller_type == 0) // NMPC(ACADOS)
        {
            if(controller.nmpc_controller_type == 0) // w_and_totalF
            {
                nmpc_params = const_params::W_TOTALF_PARAMS;
                // controller_timer = nh.createTimer(ros::Duration(0.01), fsm_cb::AcadosNmpcSimpleModelTimerCallback);
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

    AcadosNmpcWandTotalFControllerInit(nh, acados_simple_controller);

    /*--------- Client ---------*/
    arming_cmd_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    last_request = ros::Time::now().toSec();

    // fsm_ut::InitPX4(offboard_mode, land_mode, arm_cmd, disarm_cmd, setpoint_pos_pub, rate);

    w_totalF_nmpc_solver_capsule *acados_ocp_capsule = w_totalF_nmpc_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = 20;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = w_totalF_nmpc_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("w_totalF_nmpc_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = w_totalF_nmpc_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = w_totalF_nmpc_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = w_totalF_nmpc_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = w_totalF_nmpc_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = w_totalF_nmpc_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = w_totalF_nmpc_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    double lbx0[10];
    double ubx0[10];
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0.0;
    ubx0[2] = 0.0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = 1;
    ubx0[6] = 1;
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 0;
    ubx0[8] = 0;
    lbx0[9] = 0;
    ubx0[9] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[10];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 1.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;

    // initial value for control input
    double u0[4];
    u0[0] = 9.8015;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[10 * (20+1)];
    double utraj[4 * 20];
    double params[4] = {1.0, 0.0, 0.0, 0.0};

    // solve ocp in loop
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        double* yref = static_cast<double*>(calloc(13, sizeof(double)));
        // change only the non-zero elements:
        yref[2] = 1.0;
        yref[9] = 9.8015;

        for (int i = 0; i < N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
        }
        free(yref);

        double* yref_e = static_cast<double*>(calloc(9, sizeof(double)));
        // change only the non-zero elements:
        yref_e[2] = 1.0;
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
        free(yref_e);

        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
            status = w_totalF_nmpc_acados_update_params(acados_ocp_capsule, i, params, 4);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        status = w_totalF_nmpc_acados_update_params(acados_ocp_capsule, N, params, 4);
        status = w_totalF_nmpc_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*10]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*4]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( 10, N+1, xtraj, 10);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( 4, N, utraj, 4 );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("w_totalF_nmpc_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("w_totalF_nmpc_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);

    w_totalF_nmpc_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);



    // free solver
    status = w_totalF_nmpc_acados_free(acados_ocp_capsule);
    if (status) {
        printf("w_totalF_nmpc_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = w_totalF_nmpc_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("w_totalF_nmpc_acados_free_capsule() returned status %d. \n", status);
    }
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