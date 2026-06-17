/*此节点用于监听mesh节点发布的control_data/data和task_status并在到达指定任务阶段时向egoplanner发送坐标
此节点用于多机仿真，通过uav_nm、uav_nm_区分不同无人机*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h> 
#include <iostream>
#include <std_msgs/UInt8.h>

#define STRAIGHT_CTRL      1
#define EGO_CTRL           2
#define GATE_CTRL          3

std::string uav_nm ;
// std::string uav_nm_ = "uav0";

ros::Publisher straight_target_pub;
ros::Publisher ego_target_pub;
geometry_msgs::PoseStamped straight_target_msg;
geometry_msgs::PoseStamped ego_target_msg;
mavros_msgs::PositionTarget::ConstPtr pose_msg;
uint8_t flag = 0;
bool flag_ = false; 

void straight_ctrl(){
    // std::cout<<"接收到直接控制数据："<<msg->position.x<<", "<<msg->position.y<<", "<<msg->position.z<<std::endl;
    std::cout<<"进行直接控制"<<std::endl;
    // 确保有位置指令
    if(pose_msg){
        std::cout<<"发送直接控制数据："<<pose_msg->position.x<<", "<<pose_msg->position.y<<", "<<pose_msg->position.z<<std::endl;
        // 设置时间戳
        straight_target_msg.header.stamp = ros::Time::now();  // 当前时间戳
        straight_target_msg.header.frame_id = "map";  // 根据需要设置坐标系

        // 转换 PositionTarget 到 PoseStamped
        straight_target_msg.pose.position.x = pose_msg->position.x;
        straight_target_msg.pose.position.y = pose_msg->position.y;
        straight_target_msg.pose.position.z = pose_msg->position.z;

        // 姿态赋值 - 默认无旋转（四元数必须合法）
        straight_target_msg.pose.orientation.x = 0.0;
        straight_target_msg.pose.orientation.y = 0.0;
        straight_target_msg.pose.orientation.z = 0.0;
        straight_target_msg.pose.orientation.w = 1.0;

        straight_target_pub.publish(straight_target_msg);

        pose_msg.reset();  // 用完之后重置
    }
    else std::cout<<"位置指令为空！！"<<std::endl;
}

void ego_ctrl(){
    std::cout<<"进行ego控制"<<std::endl;
    // 确保有位置指令
    if(pose_msg){
        if(pose_msg->position.x != ego_target_msg.pose.position.x 
            || pose_msg->position.y != ego_target_msg.pose.position.y
            || pose_msg->position.z != ego_target_msg.pose.position.z )
            {
                std::cout<<"发送新ego控制数据："<<pose_msg->position.x<<", "<<pose_msg->position.y<<", "<<pose_msg->position.z<<std::endl;

                ego_target_msg.pose.position.x = pose_msg->position.x;
                ego_target_msg.pose.position.y = pose_msg->position.y;
                ego_target_msg.pose.position.z = pose_msg->position.z;

                ego_target_pub.publish(ego_target_msg);
            }
        pose_msg.reset();  // 用完之后重置
    }
    else std::cout<<"位置指令为空！！"<<std::endl;
}

// void PositionCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
// {
//     switch (flag)
//     {
//     case STRAIGHT_CTRL:
//         straight_ctrl(msg);
//         break;

//     case EGO_CTRL:
//         ego_ctrl(msg);
//         break;
    
//     default:
//         break;
//     }
// }

void PoseCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    std::cout<<"接收到控制数据："<<msg->position.x<<", "<<msg->position.y<<", "<<msg->position.z<<std::endl;
    pose_msg = msg;
}

void FlagCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ros::Duration(0.1).sleep();  // 等待0.1秒等待px4ctrl控制模式先更新
    // flag = msg->data;
    switch (msg->data)
    {
    case STRAIGHT_CTRL:
        straight_ctrl();
        break;

    case EGO_CTRL:
        ego_ctrl();
        break;
    
    default:
        break;
    }
}

int main(int argc, char** argv)
{
    // std::string node_name = "xyz_input_node" + uav_nm_;
    // ros::init(argc, argv, node_name.c_str());
    ros::init(argc, argv, "xyz_input_node");
    // ros::param::get("uav_num",uav_nm);
    // ros::NodeHandle nh(uav_nm);
    ros::NodeHandle nh;

    // 初始化ego目标坐标
    ego_target_msg.pose.position.x = 0.0;
    ego_target_msg.pose.position.y = 0.0;
    ego_target_msg.pose.position.z = -1.0;

    // ros::init(argc, argv, "xyz_input_node");
    // ros::NodeHandle nh(uav_nm);

    // 创建publisher，话题名为/output_target
    straight_target_pub = nh.advertise<geometry_msgs::PoseStamped>("straight_output_target", 10);
    ego_target_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_input_target", 10);

    ros::Subscriber position_sub = nh.subscribe<mavros_msgs::PositionTarget>("control_data/data", 1,PoseCallback);
    ros::Subscriber flag_sub = nh.subscribe<std_msgs::UInt8>("/control_data/task_status", 1 , FlagCallback);
    // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1 , PoseCallback);

    ros::spin();

    // while (ros::ok())
    // {
    //     double x, y, z;
    //     std::cout << "请输入目标点的XYZ坐标（空格分隔）：";
    //     std::cin >> x >> y >> z;

    //     // 创建消息指针并填充数据
    //     geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped);
    //     msg->header.stamp = ros::Time::now();
    //     msg->header.frame_id = "world"; // 根据需要指定参考系

    //     msg->pose.position.x = x;
    //     msg->pose.position.y = y;
    //     msg->pose.position.z = z;
    //     // orientation默认给0四元数
    //     msg->pose.orientation.w = 1.0;
    //     msg->pose.orientation.x = 0.0;
    //     msg->pose.orientation.y = 0.0;
    //     msg->pose.orientation.z = 0.0;

    //     // 发布消息
    //     target_pub.publish(msg);

    //     std::cout << "已发布：" << x << ", " << y << ", " << z << std::endl;

    //     ros::spinOnce();
    // }
    return 0;
}