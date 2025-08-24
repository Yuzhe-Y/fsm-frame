/**
 * @file    single_offboard_fsm
 * @brief   finite state machine for single drone
 * @author  FLAG Lab, BIT
 * @version 4.0
 * @date    2024-06-03
 */

// !!!这个头文件一定放在最前面!!!
#include "boost/sml.hpp"

#include "fsm_ctrl/nodes/single_offboard_fsm.hpp"

using namespace std;
 
#define RATE 50.0
#define INTERV 1.0/RATE
#define HEIGHT 1.0 
 
bool is_udp_enable = true;         //是否可以进行进程间通信

int cmd = 0;                                     //输入命令
int last_cmd = 0;                                //上一次输入命令

ros::Time last_request;           //主程序开始时间

/*--------------------------- ROS ---------------------------*/
/*--------- Publisher ---------*/
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher local_target_pub;
ros::Publisher local_attitude_pub;
/*--------- Subscriber ---------*/
ros::Subscriber state_sub;
ros::Subscriber voltage_sub;
ros::Subscriber position_sub;
ros::Subscriber vel_sub;
ros::Subscriber imu_sub;
ros::Subscriber rc_sub;

/*--------- Client ---------*/
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

mavros_msgs::SetMode offboard_mode;
mavros_msgs::SetMode land_mode;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd, disarm_cmd;


/*--------------------------- CMD Listener ---------------------------*/

void UdpListen(const uint16_t cport)
{
    ros::NodeHandle nh;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        ROS_ERROR("Network Error!!!");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY); // 自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if (bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
        perror("bind error:");
        exit(1);
    }

    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while (ros::ok())
    {
        char *p;
        int ent = 0;
        int user_cmd;
        double px, py, pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if (recv_num < 0 || abs(recv_num - 19) > 3)
        {
            ROS_ERROR("Receive Fail!!!");
            continue;
        }
        recv_buf[recv_num] = '\0';
        // ROS_INFO("Rec: %s, len = %d", recv_buf, recv_num);

        /* receive UDP data */
        p = strtok(recv_buf, dot);
        sscanf(p, "%d", &user_cmd);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &px);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &py);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &pz);
        
        if (is_udp_enable) {cmd = user_cmd;}
    }
}

/*--------------------------- Main ---------------------------*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle nh;

    /*--------- Publisher ---------*/
    ros::Publisher setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);
    ros::Publisher setpoint_raw_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);

    /*--------- Subscriber ---------*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, fsm_cb::MavrosStateCallback);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("/mavros/battery", 10, fsm_cb::MavrosBatteryCallback);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, fsm_cb::MavrosFcuPoseCallback);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, fsm_cb::MavrosFcuVelCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 3, fsm_cb::MavrosImuCallback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
        ("/mavros/rc/in", 10, fsm_cb::MavrosRcCallback);


    /*--------- Client ---------*/
    ros::ServiceClient arming_cmd_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode offboard_mode;
    mavros_msgs::SetMode land_mode;

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool disarm_cmd;

    ros::Rate rate(RATE); // the setpoint publishing rate must be faster than 2Hz

    ros::Time last_request = ros::Time::now();

    /*--------- PX4 initialize ---------*/
    //user command monitor on
    new std::thread(&UdpListen, 12001);

    InitPX4(offboard_mode, land_mode, arm_cmd, disarm_cmd, setpoint_pos_pub, rate);

    while (ros::ok())
    {
        ros::spinOnce();

        last_cmd = cmd; //命令保存
        if (cmd == 0)
        {
            CheckAndSwitchToOffboardAndArm(fsm_cb::mavros_state, offboard_mode, arm_cmd, 
                                           set_mode_client, arming_cmd_client, last_request);
        }

        else if (cmd == 1)
        {
            
        }
      
        else if (cmd == 2)
        {
            
        }

        else if (cmd == 3)
        {

        }

        else if (cmd == 4)
        {
        
        }

        rate.sleep();
    }
    return 0;
}