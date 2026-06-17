/*
无人机控制
0. 飞行器解锁
1. 定点飞行
在外部控制模式下进行定点飞行
2. local_position 指点飞行
以ENU（东北天）坐标系为坐标进行定点飞行
3. 速度控制飞行(多模式)
    3.1 控制Vx、Vy、偏航角Yaw  Z不变定高飞行    (推荐！！！)
    3.2 控制Vx、Vy、Vz、偏航角Yaw            （比较危险不推荐！！！）
    3.3
*/
/* Includes-------------------------------------------------------*/
#include "ros/ros.h"
#include "mavros_msgs/PositionTarget.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include "cmath"
#include "algorithm"
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#define PI 3.14159265358979323846

#define STRAIGHT_CTRL      1
#define EGO_CTRL           2
#define GATE_CTRL          3

uint8_t ctrl_mode_flag = 2;    // 代表控制模式的标志

/* Private typedef -----------------------------------------------*/
//飞行围栏（安全区域）
float safe_area_xmax;
float safe_area_ymax;
float safe_area_hightmax;
//飞行速度限制
float vx_max;float vy_max;float vz_max;

static ros::Publisher local_pose_vel_pub;  //位置和速度消息发布
static ros::ServiceClient set_mode_client; //飞行模式设置
static ros::Subscriber state_sub;          //状态订阅
static ros::Subscriber local_position_pose_sub;//位置数据订阅 local position
static ros::Subscriber control_data_sub; //地面站期望指令订阅

static ros::Subscriber straight_control_data_sub;    // 直接控制模式订阅消息
static ros::Subscriber ego_control_data_sub;    // ego控制模式订阅消息
static ros::Subscriber flag_sub;    // 控制模式订阅消息
volatile bool received_local_pose = false;  // 表明是否接收到无人机位姿初始数据

static ros::Subscriber rcin_sub;           //遥控器数据输入订阅
static ros::ServiceClient arming_client;   //解锁订阅

static mavros_msgs::SetMode offb_set_mode; //要发布的guided模式
static mavros_msgs::CommandBool arm_cmd;   //要发布的解锁模式
mavros_msgs::PositionTarget control_data; //从地面站接收到的控制指令
mavros_msgs::PositionTarget command_position_pose;          //位置信息
mavros_msgs::PositionTarget command_position_pose_and_vel;  //x y 轴速度和高度信息

static uint8_t fightarea_check_flag;     //飞行区域安全检查 0：没有异常 1：有异常
static uint8_t fightarea_flag_change_allow;     //无人机进行飞行区域检查标志位改变的权利 1：允许 0：不允许
static uint8_t mode_check_flag;          //模式安全检查    0：没有异常 1：有异常
static uint8_t mode_select_flag;         //模式切换选择     0：guided 1:guided模式下的定点 2:position模式
static uint8_t arm_select_flag;          //解锁选择       0：不解锁 1：解锁
static uint16_t rc_data_channel6_last = 0;            //记录上一时刻的遥控器通道数据 主要
static ros::Time guided_enter_time;                    // 记录进入 Guided 模式的时间
static bool guided_hold_active = false;                // 是否处于定点保持过渡期

ros::Timer timer_safe_check;        //飞行器安全检查定时器
ros::Timer timer_mode_change;       //飞行器模式切换检查定时器

static mavros_msgs::State current_state;//状态数据
static mavros_msgs::RCIn rc_data;       //遥控器数据
static geometry_msgs::PoseStamped local_pose_data;     //无人机位置数据
static mavros_msgs::PositionTarget local_pose_data_last;//无人机模式切换前的位置数据
static mavros_msgs::PositionTarget local_pose_data_boforeOutarea;//无人机飞出安全区域前的位置数据

// 互斥所
std::mutex control_data_mutex;

// 【修复】发点使能标志：切LOITER时停止发setpoint，解决切不回去问题
volatile bool send_setpoint_enable = true;

/* function define -----------------------------------------------*/
// **************** 功能函数 ****************
double getYawFromPose(const geometry_msgs::PoseStamped& pose_msg)
{
    geometry_msgs::Quaternion quat = pose_msg.pose.orientation;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat,tf_quat);

    double roll,pitch,yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll,pitch,yaw);
    return yaw;
}


//******************ROS sub 回调函数**************
//状态回调函数
void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
//遥控器输入回调函数
void rc_input_callback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    rc_data = *msg;
}
//位置信息回调函数
void localpositionposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*
        ENU坐标系 三轴朝向：前左上：东北天
    */
    received_local_pose = true;
    local_pose_data = *msg;

    /*airsim专用*/
    // // 位置，ENU → FLU
    // local_pose_data.pose.position.x = msg->pose.position.y;               // 前 (Forward)
    // local_pose_data.pose.position.y = -msg->pose.position.x;              // 左 (Left)
    // local_pose_data.pose.position.z = msg->pose.position.z;               // 上 (Up)

    // // 姿态（四元数），只做简化处理
    // local_pose_data.pose.orientation.x = msg->pose.orientation.y;
    // local_pose_data.pose.orientation.y = -msg->pose.orientation.x;
    // local_pose_data.pose.orientation.z = msg->pose.orientation.z;
    // local_pose_data.pose.orientation.w = msg->pose.orientation.w;
}

//****************** px4 控制函数 ******************
/*
guided模式下的定点飞行
输入：需要发布的位置信息，发布者
输出：0（发布失败）1（发布成功）
*/
int command_position_control(double x,double y,double hight,float yaw)
{
    //首先判断设置的飞行参数有没有超出地理围栏
    if(hight > safe_area_hightmax || fabs(x) > safe_area_xmax || fabs(y) > safe_area_ymax)
    {
        ROS_INFO("give position out of the safe_area");
        return 0;
    }
    if(mode_check_flag!=0)
    {
        ROS_INFO("mode is not guided mode");
        return 0;
    }
    if(fightarea_check_flag!=0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }

    /*airsim用*/
    // command_position_pose.position.x = -y;
    // command_position_pose.position.y = x;
    // command_position_pose.position.z = hight;
    // command_position_pose.yaw = yaw+PI/2;

    /*实机飞行用*/
    command_position_pose.position.x = x;
    command_position_pose.position.y = y;
    command_position_pose.position.z = hight;
    command_position_pose.yaw = yaw;

    local_pose_vel_pub.publish(command_position_pose);
    return 1;
}
/*
无人机x、y轴速度控制， Z轴高度控制 偏航角控制
输入：需要发布的位置信息，发布者
输出：0（发布失败）1（发布成功）
*/
int command_VelHightYaw_control(double vx,double vy,double hight,float yaw)
{
    ROS_INFO("vx:%f vy:%f hight:%f yaw:%f", vx,vy,hight,yaw);
    //首先判断设置的飞行参数有没有超出地理围栏
    if(hight > safe_area_hightmax)
    {
        ROS_INFO("give hight is too high");
        return 0;
    }
    if(mode_check_flag!=0)
    {
        ROS_INFO("mode is not guided mode");
        return 0;
    }
    if(fightarea_check_flag!=0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }
    command_position_pose_and_vel.velocity.x = vx;
    command_position_pose_and_vel.velocity.y = vy;
    command_position_pose_and_vel.position.z = hight;
    command_position_pose_and_vel.yaw = yaw;

    local_pose_vel_pub.publish(command_position_pose_and_vel);
    return 1;
}


//**********************定时器回调函数******************
/*
无人机飞行安全检查函数
如果不安全处于command_position 外部控制定点模式
*/
void safe_check_Callback(const ros::TimerEvent &event)
{
    //无人机状态检查
    switch (mode_select_flag)
    {
        case 1: //command 定点模式
            if(current_state.mode != "GUIDED")
            {
                mode_check_flag = 1;
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    ROS_INFO("guided     enable");
            }
            else
            {
                mode_check_flag = 0;
                if(arm_select_flag == 1)
                {
                    if(!current_state.armed)
                    {
                        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                            ROS_INFO("Vehicle armed");
                    }
                }
            }
            ROS_INFO("command position");
            break;
        case 2:
            //纯command模式：防止px4意外跳出guided模式，让其重新进入guided模式
            if(current_state.mode != "GUIDED")
            {
                mode_check_flag = 1;
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    ROS_INFO("guided enable");
            }
            else
            {
                mode_check_flag = 0;
                if(arm_select_flag == 1)
                {
                    if(!current_state.armed)
                    {
                        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                            ROS_INFO("Vehicle armed");
                    }
                }
            }
            ROS_INFO("command control");
            break;
    }

    //飞行区域安全检查
    if(fabs(local_pose_data.pose.position.x) > safe_area_xmax || fabs(local_pose_data.pose.position.y) > safe_area_ymax
        || fabs(local_pose_data.pose.position.z) > safe_area_hightmax)
    {
        if(fightarea_flag_change_allow)
        {
            fightarea_check_flag = 1;
            fightarea_flag_change_allow = 0;
            send_setpoint_enable = false;  // 越界停止发点
            ROS_ERROR("fightarea_check_flag change to 0");
        }
        ROS_ERROR("The UAV is out of the safe area");
    }
    else
    {
        if(fightarea_flag_change_allow)
        {
            local_pose_data_boforeOutarea.position.x = local_pose_data.pose.position.x;
            local_pose_data_boforeOutarea.position.y = local_pose_data.pose.position.y;
            local_pose_data_boforeOutarea.position.z = local_pose_data.pose.position.z;
            local_pose_data_boforeOutarea.yaw = getYawFromPose(local_pose_data);
            fightarea_check_flag = 0;
            ROS_ERROR("The UAV is in the safe area");
        }

    }
}

void mode_check_Callback(const ros::TimerEvent &event)
{
    static uint16_t rc_data_channel5_last = 0;

    // 检测 ch5 是否有变化
    if (!((rc_data.channels[4] > rc_data_channel5_last - 20) &&
          (rc_data.channels[4] < rc_data_channel5_last + 20)))
    {
        ROS_INFO("mode change detected (ch5 changed from %d to %d)",
                 rc_data_channel5_last, rc_data.channels[4]);

        local_pose_data_last.position.x = local_pose_data.pose.position.x;
        local_pose_data_last.position.y = local_pose_data.pose.position.y;
        local_pose_data_last.position.z = local_pose_data.pose.position.z;
        local_pose_data_last.yaw = getYawFromPose(local_pose_data);

        control_data.position.x = local_pose_data.pose.position.x;
        control_data.position.y = local_pose_data.pose.position.y;
        control_data.position.z = local_pose_data.pose.position.z;

        // 【核心修复】模式切换逻辑
        if (rc_data.channels[4] >= 1850)
        {
            // GUIDED 模式
            mode_select_flag = 1;
            offb_set_mode.request.custom_mode = "GUIDED";
            send_setpoint_enable = true;   // 恢复发点
            guided_enter_time = ros::Time::now();
            guided_hold_active = true;
            ROS_INFO("Mode: GUIDED HOLD (case 1)");
        }
        else
        {
            // LOITER 模式：停止发点 + 强制切换
            mode_select_flag = 0;
            offb_set_mode.request.custom_mode = "LOITER";
            send_setpoint_enable = false;  // 停止发setpoint，释放控制权
            guided_hold_active = false;

            // 强制调用切换
            if (set_mode_client.call(offb_set_mode)) {
                ROS_INFO("Switch to LOITER success!");
            } else {
                ROS_WARN("Switch to LOITER failed");
            }
            ROS_INFO("Mode: LOITER");
        }

        fightarea_flag_change_allow = 1;
        fightarea_check_flag = 0;
    }

    // 自动过渡：3秒后进入指令跟随
    if (mode_select_flag == 1 && guided_hold_active)
    {
        double elapsed = (ros::Time::now() - guided_enter_time).toSec();
        if (elapsed >= 3.0)
        {
            mode_select_flag = 2;
            guided_hold_active = false;
            ROS_INFO("=== HOLD -> COMMAND FOLLOWING ===");
        }
    }

    rc_data_channel5_last = rc_data.channels[4];

    // 通道6解锁/上锁
    if (rc_data.channels[5] >= 1850)
        arm_select_flag = 0;
    else
        arm_select_flag = 1;

    ROS_INFO_THROTTLE(1, "arm:%d mode:%d send:%d ch5=%d ch6=%d",
                      arm_select_flag, mode_select_flag, send_setpoint_enable,
                      rc_data.channels[4], rc_data.channels[5]);
}

/*直接控制模式的回调函数*/
void straightControldataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Get control straight ctrl data");
    if(ctrl_mode_flag == STRAIGHT_CTRL){
        std::lock_guard<std::mutex> lock(control_data_mutex);
        control_data.position.x = msg->pose.position.x;
        control_data.position.y = msg->pose.position.y;
        control_data.position.z = msg->pose.position.z;
        ROS_INFO("Straight ctrl data accept");
        }
}

/*ego控制模式回调函数*/
void EGOControldataCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    ROS_INFO("Get EGO ctrl data");
    if(ctrl_mode_flag == EGO_CTRL){
        std::lock_guard<std::mutex> lock(control_data_mutex);
        control_data.position.x=msg->position.x;
        control_data.position.y=msg->position.y;
        control_data.position.z=msg->position.z;
        control_data.yaw=msg->yaw;
        ROS_INFO("ego ctrl data accept");
        }
}

/*控制模式回调*/
void FlagCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ctrl_mode_flag = msg->data;
}

void velocity_and_position_limit(mavros_msgs::PositionTarget& msg)
{
    if (msg.velocity.x > 0 && msg.velocity.x > vx_max)
        msg.velocity.x = vx_max;
    else if (msg.velocity.x < 0 && msg.velocity.x < -vx_max)
        msg.velocity.x = -vx_max;
    if (msg.velocity.y > 0 && msg.velocity.y > vy_max)
        msg.velocity.y = vy_max;
    else if (msg.velocity.y < 0 && msg.velocity.y < -vy_max)
        msg.velocity.y = -vy_max;
    if (msg.velocity.z > 0 && msg.velocity.z > vz_max)
        msg.velocity.z = vz_max;
    else if (msg.velocity.z < 0 && msg.velocity.z < -vz_max)
        msg.velocity.z = -vz_max;

}

/*
px4 control初始化函数
*/
int px4control_init(ros::NodeHandle nh)
{
    /*****发布的消息初始化***********************************************************/
    //guided模式消息
    offb_set_mode.request.custom_mode = "GUIDED";
    //解锁消息
    arm_cmd.request.value = true;
    //发布位置消息的初始化
    command_position_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    command_position_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ|
                                        mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ;

    command_position_pose.position.x = 0;
    command_position_pose.position.y = 0;
    command_position_pose.position.z = 0;


    local_pose_data_last.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_last.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_last.position.x = 0;
    local_pose_data_last.position.y = 0;
    local_pose_data_last.position.z = 0;

    local_pose_data_boforeOutarea.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_boforeOutarea.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_boforeOutarea.position.x = 0;
    local_pose_data_boforeOutarea.position.y = 0;
    local_pose_data_boforeOutarea.position.z = 0;

    //发布高度和速度消息初始化
    command_position_pose_and_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    command_position_pose_and_vel.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                                mavros_msgs::PositionTarget::IGNORE_PX |
                                                mavros_msgs::PositionTarget::IGNORE_PY |
                                                mavros_msgs::PositionTarget::IGNORE_VZ;


    command_position_pose_and_vel.position.z = 0;
    command_position_pose_and_vel.velocity.x = 0;
    command_position_pose_and_vel.velocity.y = 0;

    //控制指令数据初始化
    control_data.position.x = 0;control_data.position.y = 0;control_data.position.z = 0;

    /*****相关定时器回调函数定义******************************************************/
    timer_safe_check = nh.createTimer(ros::Duration(0.2),safe_check_Callback);
    timer_safe_check.stop();
    timer_mode_change = nh.createTimer(ros::Duration(0.1),mode_check_Callback);
    timer_mode_change.stop();

    /*****初始化的一些数据读**********************************************************/
    ros::param::get("flight_area/flight_area_xmax",safe_area_xmax);ROS_INFO("safe_area_xmax:%f",safe_area_xmax);
    ros::param::get("flight_area/flight_area_ymax",safe_area_ymax);ROS_INFO("safe_area_ymax:%f",safe_area_ymax);
    ros::param::get("flight_area/flight_area_hightmax",safe_area_hightmax);ROS_INFO("safe_area_hightmax:%f",safe_area_hightmax);
    ros::param::get("flight_velocity/vx",vx_max);ROS_INFO("vx_max:%f",vx_max);
    ros::param::get("flight_velocity/vy",vy_max);ROS_INFO("vy_max:%f",vy_max);
    ros::param::get("flight_velocity/vz",vz_max);ROS_INFO("vz_max:%f",vz_max);


    /*****初始化一些状态数据**********************************************************/
    fightarea_check_flag = 1;
    fightarea_flag_change_allow = 1;
    mode_check_flag = 1;
    mode_select_flag = 0;

    /*****ROS句柄、订阅、发布、服务器初始化*********************************************/
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_callback);
    rcin_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in",10,rc_input_callback);
    local_position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,localpositionposeCallback);

    straight_control_data_sub = nh.subscribe<geometry_msgs::PoseStamped>("straight_output_target",2,straightControldataCallback);
    ego_control_data_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd",2,EGOControldataCallback);
    flag_sub = nh.subscribe<std_msgs::UInt8>("/control_data/task_status", 1 , FlagCallback);

    // 等待收到mavros/local_position/pose初始位姿
    ROS_INFO("等待接收初始位姿...");
    ros::Rate rate(5);
    while (ros::ok() && !received_local_pose) {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout<<"初始位姿："<<local_pose_data.pose.position.x<<" "<<local_pose_data.pose.position.y<<" "<<local_pose_data.pose.position.z<<std::endl;

    control_data.position.x = command_position_pose.position.x = local_pose_data.pose.position.x;
    control_data.position.y = command_position_pose.position.y = local_pose_data.pose.position.y;
    control_data.position.z = command_position_pose.position.z = local_pose_data.pose.position.z;
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(local_pose_data.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    control_data.yaw = command_position_pose.yaw = yaw;
    ROS_INFO("已接收初始位姿，继续后续初始化!");

    local_pose_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    return 1;
}
/*
px4_control start函数
任务：MAVROS连接 飞行安全检查的初始化
*/
int px4control_start()
{
    ros::Rate rate(20);
    while(ros::ok &&!current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("connected with px4");
    timer_mode_change.start();
    ros::Duration du(0.5);
    du.sleep();
    ROS_INFO("mode change check start");

    timer_safe_check.start();
    du.sleep();
    return 1;
}


void uavcontrol()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        // 【修复】只有使能时才发点
        if (!send_setpoint_enable) {
            rate.sleep();
            continue;
        }

        switch (mode_select_flag)
        {
            case 1:
                local_pose_vel_pub.publish(local_pose_data_last);
                break;
            case 2:
                std::unique_lock<std::mutex> lock(control_data_mutex);
                velocity_and_position_limit(control_data);
                command_position_control(control_data.position.x,control_data.position.y,control_data.position.z,control_data.yaw);
                lock.unlock();
                ROS_INFO("give command to uav");
                break;
        }
        rate.sleep();
    }
}


int main(int argc,char *argv[])
{
    ros::init(argc,argv,"px4_control");
    ros::NodeHandle nh;

    ROS_INFO("OK");
    px4control_init(nh);
    ROS_INFO("contrl init sucess");
    while (! px4control_start());
    ROS_INFO("control start sucess");

    std::thread control_thread(uavcontrol);

    ros::spin();
}