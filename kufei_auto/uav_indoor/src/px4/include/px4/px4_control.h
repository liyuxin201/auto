// #ifndef PX4_CONTROL_H
// #define PX4_control_H

// #include "ros/ros.h"
// #include "mavros_msgs/PositionTarget.h"
// #include "geometry_msgs/PoseStamped.h"
// #include <geometry_msgs/TwistStamped.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/RCIn.h>
// #include <sensor_msgs/BatteryState.h>
// #include <sensor_msgs/MagneticField.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <sensor_msgs/Imu.h>

// namespace px4
// {
//     class control
//     {
//         private:


            
//             // ros::NodeHandle control_nh;         //ROS句柄

//             static ros::Publisher local_pose_pub;      //位置和速度消息发布
//             static ros::ServiceClient set_mode_client; //飞行模式设置
//             static ros::Subscriber state_sub;          //状态订阅
//             static ros::Subscriber local_position_pose_sub;//位置数据订阅 local position
            
//             static ros::Subscriber rcin_sub;           //遥控器数据输入订阅
//             static ros::ServiceClient arming_client;   //解锁订阅

//             static mavros_msgs::SetMode offb_set_mode; //要发布的offboard模式
//             static mavros_msgs::CommandBool arm_cmd;   //要发布的解锁模式
//             mavros_msgs::PositionTarget command_position_pose;          //位置信息
//             mavros_msgs::PositionTarget command_position_pose_and_vel;  //x y 轴速度和高度信息

//             static uint8_t fightarea_check_flag;     //飞行区域安全检查 0：没有异常 1：有异常
//             static uint8_t mode_check_flag;          //模式安全检查    0：没有异常 1：有异常
//             static uint8_t mode_select_flag;         //模式切换选择     0：offboard 1:offboard模式下的定点 2:position模式
//             static uint16_t rc_data_channel6_last;            //记录上一时刻的遥控器通道数据 主要用来记录位置使用

//             ros::Timer timer_safe_check;        //飞行器安全检查定时器
//             ros::Timer timer_mode_change;       //飞行器模式切换检查定时器

//             static void state_callback(const mavros_msgs::State::ConstPtr& msg);
//             static void rc_input_callback(const mavros_msgs::RCIn::ConstPtr& msg);
//             static void localpositionposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

//             static void safe_check_Callback(const ros::TimerEvent &event); //飞行器安全检查回调函数
//             static void mode_check_Callback(const ros::TimerEvent &event); //模式切换检查函数
            
//         public:
//             int init(ros::NodeHandle nh);
//             int start();
//             int command_position_control(double x,double y,double hight,float yaw);
//             int command_VelHightYaw_control(double vx,double vy,double hight,float yaw);

//             static mavros_msgs::State current_state;//状态数据
//             static mavros_msgs::RCIn rc_data;       //遥控器数据
//             static geometry_msgs::PoseStamped local_pose_data;     //无人机位置数据
//             static geometry_msgs::PoseStamped local_pose_data_last;//无人机模式切换前的位置数据

//     };

//     class uav
//     {
//         private:
            

//         public:


//     };

//     // uint8_t abc = 0;

// }


// #endif 