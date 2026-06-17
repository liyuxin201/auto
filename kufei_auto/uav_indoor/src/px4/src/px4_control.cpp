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
// #include <Eigen/Geometry>

// bool init_pose_flag=0;
// Eigen::Quaterniond q_enu(1, 0, 0, 0);
// double yaw_enu=0;
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


static ros::Subscriber rcin_sub;           //遥控器数据输入订阅
static ros::ServiceClient arming_client;   //解锁订阅

static mavros_msgs::SetMode offb_set_mode; //要发布的offboard模式
static mavros_msgs::CommandBool arm_cmd;   //要发布的解锁模式
mavros_msgs::PositionTarget control_data; //从地面站接收到的控制指令
mavros_msgs::PositionTarget command_position_pose;          //位置信息
mavros_msgs::PositionTarget command_position_pose_and_vel;  //x y 轴速度和高度信息

static uint8_t fightarea_check_flag;     //飞行区域安全检查 0：没有异常 1：有异常
static uint8_t fightarea_flag_change_allow;     //无人机进行飞行区域检查标志位改变的权利 1：允许 0：不允许
static uint8_t mode_check_flag;          //模式安全检查    0：没有异常 1：有异常
static uint8_t mode_select_flag;         //模式切换选择     0：offboard 1:offboard模式下的定点 2:position模式
static uint8_t arm_select_flag;          //解锁选择       0：不解锁 1：解锁
static uint16_t rc_data_channel6_last = 0;            //记录上一时刻的遥控器通道数据 主要用来记录位置使用

ros::Timer timer_safe_check;        //飞行器安全检查定时器
ros::Timer timer_mode_change;       //飞行器模式切换检查定时器

static mavros_msgs::State current_state;//状态数据
static mavros_msgs::RCIn rc_data;       //遥控器数据
static geometry_msgs::PoseStamped local_pose_data;     //无人机位置数据
static mavros_msgs::PositionTarget local_pose_data_last;//无人机模式切换前的位置数据
static mavros_msgs::PositionTarget local_pose_data_boforeOutarea;//无人机飞出安全区域前的位置数据

// 互斥所
std::mutex control_data_mutex; //

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
    // if(!init_pose_flag){
    //     q_enu = Eigen::Quaterniond(
    //         msg->pose.orientation.w, 
    //         msg->pose.orientation.x, 
    //         msg->pose.orientation.y,
    //         msg->pose.orientation.z
    //     );
    //     Eigen::Matrix3d rot_enu = q_enu.toRotationMatrix();
    //     Eigen::Vector3d euler = rot_enu.eulerAngles(0, 1, 2); // roll/pitch/yaw
    //     yaw_enu = euler(2);
    //     init_pose_flag = 1;
    // }
    /*
        ENU坐标系 三轴朝向：前左上：东北天
    */
    local_pose_data = *msg;
}

//****************** px4 控制函数 ******************
/*
offboard模式下的定点飞行
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
        ROS_INFO("mode is not offboard mode");
        return 0;
    }
    if(fightarea_check_flag!=0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea); //give position out of the safe_area
        return 0;
    }
    // Eigen::Vector3d target_point(x, y, z);
    // target_point = q_enu * target_point;
    // x = target_point[0]; y = target_point[1]; z = target_point[2];
    // yaw += yaw_enu;
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
        ROS_INFO("mode is not offboard mode");
        return 0;
    }
    if(fightarea_check_flag!=0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea); //give  position out of safe_area
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
        // case 0: //定点模式
        //     if(current_state.mode != "POSCTL")
        //     {
        //         if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        //             ROS_INFO("position enable");
        //     }
        //     else
        //     {
        //         if(arm_select_flag == 1)
        //         {
        //             if(!current_state.armed)
        //             {
        //                 if(arming_client.call(arm_cmd) && arm_cmd.response.success)
        //                     ROS_INFO("Vehicle armed");
        //             }
        //         }
        //     }
        //     break;
        case 1: //command 定点模式
            if(current_state.mode != "OFFBOARD")
            {
                mode_check_flag = 1;
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    ROS_INFO("offboard enable");
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
            //纯command模式：防止px4意外跳出offboard模式，让其重新进入offboard模式
            if(current_state.mode != "OFFBOARD")
            {
                mode_check_flag = 1;
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    ROS_INFO("offboard enable");
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
            fightarea_check_flag = 1; //超出边界范围 command模式下的定点模式
            fightarea_flag_change_allow = 0; //超出边界范围时，不允许再进行飞行区域标志为改变
            ROS_ERROR("fightarea_check_flag change to 0");
        }
        ROS_ERROR("The UAV is out of the safe area");
        // ROS_INFO("The UAV is out of the safe area,now change to the command position");
    }
    else
    {
        if(fightarea_flag_change_allow)
        {
            //记录下没有飞出安全区域前的位置信息
            local_pose_data_boforeOutarea.position.x = local_pose_data.pose.position.x;
            local_pose_data_boforeOutarea.position.y = local_pose_data.pose.position.y;
            local_pose_data_boforeOutarea.position.z = local_pose_data.pose.position.z;
            local_pose_data_boforeOutarea.yaw = getYawFromPose(local_pose_data);
            fightarea_check_flag = 0; //正常command控制
            ROS_ERROR("The UAV is in the safe area");
        }

    } 

    if(fightarea_check_flag != 0)
    {
        //调用最原本的位置消息发布，而不使用command_position_control,防止特殊情况已经发生而无法定点。
    }
}

void mode_check_Callback(const ros::TimerEvent &event)
{
    //飞行模式判断选择
    if(rc_data.channels[6] != rc_data_channel6_last)
    {
        ROS_INFO("mode change");
        local_pose_data_last.position.x = local_pose_data.pose.position.x;
        local_pose_data_last.position.y = local_pose_data.pose.position.y;
        local_pose_data_last.position.z = local_pose_data.pose.position.z;
        local_pose_data_last.yaw = getYawFromPose(local_pose_data);
        // switch (rc_data.channels[6])
        // {
        //     case 993: mode_select_flag = 0;offb_set_mode.request.custom_mode = "POSCTL";ROS_INFO("position");fightarea_flag_change_allow = 1; fightarea_check_flag = 0; break;
        //     case 1497: mode_select_flag = 1;offb_set_mode.request.custom_mode = "OFFBOARD";ROS_INFO("command position");fightarea_flag_change_allow = 1; fightarea_check_flag = 0; break;
        //     case 1994: mode_select_flag = 2;offb_set_mode.request.custom_mode = "OFFBOARD";ROS_INFO("command");break;
        // }
        if (rc_data.channels[6] > 900 and rc_data.channels[6] < 1200)
        {
            mode_select_flag = 0;offb_set_mode.request.custom_mode = "POSCTL";ROS_INFO("position");fightarea_flag_change_allow = 1; fightarea_check_flag = 0; 
        } else if(rc_data.channels[6] > 1300 and rc_data.channels[6] < 1600 and rc_data.channels[5] > 1700 and rc_data.channels[5] < 2100)
        {
            mode_select_flag = 1;offb_set_mode.request.custom_mode = "OFFBOARD";ROS_INFO("command position");fightarea_flag_change_allow = 1; fightarea_check_flag = 0;
        } else if(rc_data.channels[6] > 1700 and rc_data.channels[6] < 2100 and rc_data.channels[5] > 1700 and rc_data.channels[5] < 2100)
        {
            mode_select_flag = 2;offb_set_mode.request.custom_mode = "OFFBOARD";ROS_INFO("command");
        } else
        {
            mode_select_flag = 0;offb_set_mode.request.custom_mode = "POSCTL";ROS_INFO("position");fightarea_flag_change_allow = 1; fightarea_check_flag = 0;
        }
    }

    rc_data_channel6_last = rc_data.channels[6];

    //解锁 arm判断
    switch (rc_data.channels[4])
    {
        case 1000:
            arm_select_flag = 0;break; //不解锁
        case 2000:
            arm_select_flag = 1;break; //解锁
    }
    ROS_INFO("arm flag:%d, mode select flag:%d",arm_select_flag,mode_select_flag);
}

/*
地面站期望控制数据接收
*/
void controldataCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(control_data_mutex);
    control_data = *msg;
    ROS_ERROR("get control data");
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
    /*****ROS句柄、订阅、发布、服务器初始化*********************************************/
    //订阅消息初始化
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_callback);
    rcin_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in",10,rc_input_callback);
    local_position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,localpositionposeCallback);

    control_data_sub = nh.subscribe<mavros_msgs::PositionTarget>("control_data/data",2,controldataCallback);
    
    //发布消息初始化
    local_pose_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10);
    
    //客户端初始化
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    /*****发布的消息初始化***********************************************************/
    //offboard模式消息
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //解锁消息
    arm_cmd.request.value = true;
    //发布位置消息的初始化
    command_position_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //坐标设置NED
    command_position_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ|
                                        mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ;

    command_position_pose.position.x = 0;
    command_position_pose.position.y = 0;
    command_position_pose.position.z = 0;


    local_pose_data_last.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //坐标设置NED
    local_pose_data_last.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_last.position.x = 0;
    local_pose_data_last.position.y = 0;
    local_pose_data_last.position.z = 0;

    local_pose_data_boforeOutarea.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //坐标设置NED
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
    command_position_pose_and_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //坐标设置NED
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
    
    ROS_INFO("1");

    /*****相关定时器回调函数定义******************************************************/
    //飞行器安全检查回调函数
    timer_safe_check = nh.createTimer(ros::Duration(0.2),safe_check_Callback); //200ms检查一次
    timer_safe_check.stop();//关闭定时器
    //飞行器模式切换检查回调函数
    timer_mode_change = nh.createTimer(ros::Duration(0.1),mode_check_Callback); //100ms检查一次
    timer_mode_change.stop();//关闭定时器
    ROS_INFO("1");
    /*****初始化的一些数据读**********************************************************/
    ros::param::get("flight_area/flight_area_xmax",safe_area_xmax);ROS_INFO("safe_area_xmax:%f",safe_area_xmax);
    ros::param::get("flight_area/flight_area_ymax",safe_area_ymax);ROS_INFO("safe_area_ymax:%f",safe_area_ymax);
    ros::param::get("flight_area/flight_area_hightmax",safe_area_hightmax);ROS_INFO("safe_area_hightmax:%f",safe_area_hightmax);
    ros::param::get("flight_velocity/vx",vx_max);ROS_INFO("vx_max:%f",vx_max);
    ros::param::get("flight_velocity/vy",vy_max);ROS_INFO("vy_max:%f",vy_max);
    ros::param::get("flight_velocity/vz",vz_max);ROS_INFO("vz_max:%f",vz_max);
    

    /*****初始化一些状态数据**********************************************************/
    fightarea_check_flag = 1; //默认超出边界
    fightarea_flag_change_allow = 1; //默认允许进行飞行区域安全检查
    mode_check_flag = 1;      //默认没有进入offboard模式
    mode_select_flag = 0;     //默认处于定点模式

    return 1;
}
/*
px4_control start函数
任务：MAVROS连接 飞行安全检查的初始化 
*/
int px4control_start()
{
    //PX4连接等待
    ros::Rate rate(20);
    while(ros::ok &&!current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("connected with px4");
    //飞行器模式切换回调函数开启
    timer_mode_change.start();
    ros::Duration du(0.5);
    du.sleep();             //延时0.5秒，确保读取到遥控器数据数据
    ROS_INFO("mode change check start");

    //飞行器安全检查回调函数开启
    timer_safe_check.start();
    du.sleep();
    return 1;
}


void uavcontrol()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        switch (mode_select_flag)
        {
            case 1:
                //调用最原本的位置消息发布，而不使用command_position_control,防止特殊情况已经发生而无法定点。

                local_pose_vel_pub.publish(local_pose_data_last);
                break;
            case 2:
                std::unique_lock<std::mutex> lock(control_data_mutex); //获取锁
                velocity_and_position_limit(control_data); //对速度进行限制
                command_position_control(control_data.position.x,control_data.position.y,control_data.position.z,control_data.yaw);
                lock.unlock(); // 释放锁
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