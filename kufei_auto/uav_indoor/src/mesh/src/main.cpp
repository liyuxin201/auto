#include <thread>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <sstream>
// #include <serial/serial.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/UInt8.h>
#include <iomanip>

#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "mesh/uavs.h"
// -------- tcp include ------
#include "mesh/tcp_server.h"

// -------------------------- UAV data start ------------------------------
mavros_msgs::PositionTarget control_data;                   //从地面站接收到的控制指令
mavros_msgs::PositionTarget command_position_pose;          //位置信息
mavros_msgs::PositionTarget command_position_pose_and_vel;  //x y 轴速度和高度信息
static ros::Subscriber local_position_pose_sub;//位置数据订阅 local position

static ros::Publisher ControlCommand_pub;///控制指令速度消息发布
static ros::Publisher taskStatus_pub;

static ros::Publisher uavsPose_pub;
static ros::Publisher video_control_pub; // 视频流控制消息发布

static ros::Publisher local_pose_vel_pub;  //位置和速度消息发布
static ros::ServiceClient arming_client;   //解锁订阅
static ros::ServiceClient set_mode_client; //飞行模式设置
static mavros_msgs::SetMode offb_set_mode; //要发布的offboard模式
static mavros_msgs::CommandBool arm_cmd;   //要发布的解锁模式
mavros_msgs::State current_state; ///无人机当前的状态参数


//飞行速度限制
float vx_max =15;
float vy_max =15;
float vz_max =3;

int uav_id = 0 ;         //无人机ID 1 2 3 4 5 6 7
//修改参数来匹配无人机接收控制指令---------------------------//
const uint8_t FRAME_HEADER = 0xAA;
const uint8_t FRAME_TAIL = 0xFF;
const size_t FRAME_SIZE = 14; // 总帧大小：帧头 + 数据 + 帧尾
const size_t DATA_SIZE = 12; // 数据部分大小
const size_t BUFFER_SIZE = 256; // 缓冲区大小

uint8_t control_command[5]; ///control_command[0]作为标识位 判别是位置控制还是速度控制

uint8_t SendData_array[60];
float RosFloatData_array[13];
float frame_data[3];

uint8_t video_control_flag;

//无人机接收的数据格式
struct ParsedData {
    uint8_t droneID;
    uint8_t status;    // 状态
    float expectedX;       // 期望位置X
    float expectedY;       // 期望位置Y
    float expectedZ;       // 期望位置Z
    float yaw;             // 偏航
    std::string locationSource; // 定位来源

    // float uav0_x; float uav0_y;
    // float uav1_x; float uav1_y;
    // float uav2_x; float uav2_y;
    // float uav3_x; float uav3_y;
    // float uav4_x; float uav4_y;
    // float uav5_x; float uav5_y;
    // float uav6_x; float uav6_y;
    // float uav7_x; float uav7_y;
    // float uav8_x; float uav8_y;
    // float uav9_x; float uav9_y;
};


// 定义飞行模式字符串
const std::string MODE_PX4_MANUAL = "MANUAL";
const std::string MODE_PX4_ACRO = "ACRO";
const std::string MODE_PX4_ALTITUDE = "ALTCTL";
const std::string MODE_PX4_POSITION = "POSCTL";
const std::string MODE_PX4_OFFBOARD = "OFFBOARD";
const std::string MODE_PX4_STABILIZED = "STABILIZED";
const std::string MODE_PX4_RATTITUDE = "RATTITUDE";

// Tcp Server
// TcpServer tcp_server("192.168.42.62", 10000);
TcpServer* tcp_server = nullptr;
// -------------------------- UAV data end ------------------------------


// -------------------------- Function start ----------------------------
std::string floatToHexIEEE754(float value) {
    uint32_t binaryValue;
    memcpy(&binaryValue, &value, sizeof(float));
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(8) << std::hex << binaryValue;
    return ss.str();
}

//转十六进制
std::string byteToHex(uint8_t byte) 
{
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte);
    return ss.str();
}

//uint8转化字符串
std::string concatUInt8ToString(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return byteToHex(a) + byteToHex(b) + byteToHex(c) + byteToHex(d);
}

// 将符合 IEEE 754 标准的字节数组转换为 float 类型的值
float hexToFloatIEEE754(const std::vector<uint8_t>& bytes) 
{
    static_assert(sizeof(float) == 4, "Size of float should be 4 bytes"); // 确认 float 的大小
    float value;
    std::memcpy(&value, bytes.data(), sizeof(float)); // 从字节数组复制到 float
    return value;  // 返回浮点值
}


// 新增函数：将两个十六进制数表示的整数值转换为浮点数并除以100得到实际位置
float hexToFloatPosition(const std::vector<uint8_t>& bytes) 
{
    if (bytes.size() != 2) {
        // 如果不是两个字节，返回0或者处理错误
        return 0.0f;
    }
    
    // 将两个字节组合成一个16位整数（大端序）
    int16_t combinedValue = (static_cast<int16_t>(bytes[0]) << 8) | static_cast<int16_t>(bytes[1]);
    
    // 转换为浮点数并除以100得到实际位置
    float position = static_cast<float>(combinedValue) / 100.0f;
    
    return position;
}


// 将float转换为4个字节
void floatToBytes(float value, uint8_t* bytes) {
    memcpy(bytes, &value, 4);
}

// 打包SendData_array数据
void packSendData() {
    SendData_array[0] = 0xAA;  // 帧头
    SendData_array[1] = uav_id;  // 无人机ID
    
    // 打包float数据到字节数组 (从索引6开始，每个float占4字节)
    int index = 6;
    for (int i = 0; i < 13; i++) {
        floatToBytes(RosFloatData_array[i], &SendData_array[index]);
        index += 4;
    }
    
    SendData_array[59] = 0xFF;  // 帧尾
}

// 解析接收到的控制指令
void parseControlCommand(const std::vector<uint8_t>& data) {
    ParsedData result;

    // 打印接收到的字节数据
    // ROS_INFO("Received data bytes:");  
    // for (size_t i = 0; i < data.size(); ++i) {  
    //     ROS_INFO("Byte %zu: 0x%02X", i, data[i]);  // 打印字节，格式为十六进制  
    // }  

    // 无人机序号 (19)
    result.droneID = data[18]; // 无人机序号

    if (result.droneID != uav_id) {
        ROS_WARN("Drone serial number mismatch my:%d recieve:%d", uav_id, result.droneID);
        return; // 不匹配则返回，跳过数据解析
    }

    // 状态 (01)
    result.status = data[1]; // 状态（0: 其他, 1: 准备阶段, 2: 对穿阶段, 3: 集群阶段, 4: 穿门）

    // 期望位置X (02-05)
    result.expectedX = hexToFloatIEEE754({data.begin() + 2, data.begin() + 6});

    // 期望位置Y (06-09)
    result.expectedY = hexToFloatIEEE754({data.begin() + 6, data.begin() + 10});
    
    // 期望位置Z (10-13)
    result.expectedZ = hexToFloatIEEE754({data.begin() + 10, data.begin() + 14});
    
    // 偏航 (14-17)
    result.yaw = hexToFloatIEEE754({data.begin() + 14, data.begin() + 18});
    
    // 定位来源 (18)
    uint8_t locationSource = data[18];
    switch (locationSource) {
        case 0: result.locationSource = "GPS"; break;
        case 1: result.locationSource = "VISION"; break;
        case 2: result.locationSource = "FUSION（GPS+VISION）"; break;
        default: result.locationSource = "unknow"; break;
    }

    mesh::uavs uavs_msg;
    uavs_msg.uav0_x = hexToFloatPosition({data.begin() + 19, data.begin() + 21});
    uavs_msg.uav0_y = hexToFloatPosition({data.begin() + 21, data.begin() + 23});
    uavs_msg.uav1_x = hexToFloatPosition({data.begin() + 23, data.begin() + 25});
    uavs_msg.uav1_y = hexToFloatPosition({data.begin() + 25, data.begin() + 27});
    uavs_msg.uav2_x = hexToFloatPosition({data.begin() + 27, data.begin() + 29});
    uavs_msg.uav2_y = hexToFloatPosition({data.begin() + 29, data.begin() + 31});
    uavs_msg.uav3_x = hexToFloatPosition({data.begin() + 31, data.begin() + 33});
    uavs_msg.uav3_y = hexToFloatPosition({data.begin() + 33, data.begin() + 35});
    uavs_msg.uav4_x = hexToFloatPosition({data.begin() + 35, data.begin() + 37});
    uavs_msg.uav4_y = hexToFloatPosition({data.begin() + 37, data.begin() + 39});
    uavs_msg.uav5_x = hexToFloatPosition({data.begin() + 39, data.begin() + 41});
    uavs_msg.uav5_y = hexToFloatPosition({data.begin() + 41, data.begin() + 43});
    uavs_msg.uav6_x = hexToFloatPosition({data.begin() + 43, data.begin() + 45});
    uavs_msg.uav6_y = hexToFloatPosition({data.begin() + 45, data.begin() + 47});
    uavs_msg.uav7_x = hexToFloatPosition({data.begin() + 47, data.begin() + 49});
    uavs_msg.uav7_y = hexToFloatPosition({data.begin() + 49, data.begin() + 51});
    uavs_msg.uav8_x = hexToFloatPosition({data.begin() + 51, data.begin() + 53});
    uavs_msg.uav8_y = hexToFloatPosition({data.begin() + 53, data.begin() + 55});
    uavs_msg.uav9_x = hexToFloatPosition({data.begin() + 55, data.begin() + 57});
    uavs_msg.uav9_y = hexToFloatPosition({data.begin() + 57, data.begin() + 59});

    video_control_flag = data[59];

    // 发布期望位置数据
    mavros_msgs::PositionTarget positionMsg;
    std_msgs::UInt8 task_status_msg;

    positionMsg.header.stamp = ros::Time::now();
    positionMsg.header.frame_id = "base_link"; // 设置坐标系
    positionMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    positionMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                            mavros_msgs::PositionTarget::IGNORE_VY | 
                            mavros_msgs::PositionTarget::IGNORE_VZ | 
                            mavros_msgs::PositionTarget::IGNORE_AFX | 
                            mavros_msgs::PositionTarget::IGNORE_AFY | 
                            mavros_msgs::PositionTarget::IGNORE_AFZ ;
    
    positionMsg.position.x = result.expectedX;
    positionMsg.position.y = result.expectedY;
    positionMsg.position.z = result.expectedZ;
    positionMsg.yaw = result.yaw;

    ControlCommand_pub.publish(positionMsg);

    task_status_msg.data = result.status;
    taskStatus_pub.publish(task_status_msg);
    ROS_INFO("position: X = %.2f, Y = %.2f, Z = %.2f, Yaw = %.2f",
              result.expectedX, result.expectedY, result.expectedZ, result.yaw);

    // 发布自己和其他无人机的位置
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "world";
    cloud->height = 1;
    
    // 添加除自己外的其他无人机位置到点云（自己是uav_id，假设从0开始）
    // uav0 对应 ID 0, uav1 对应 ID 1, ..., uav9 对应 ID 9
    if (uav_id != 0) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav0_x, uavs_msg.uav0_y, 0.0));
    }
    if (uav_id != 1) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav1_x, uavs_msg.uav1_y, 0.0));
    }
    if (uav_id != 2) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav2_x, uavs_msg.uav2_y, 0.0));
    }
    if (uav_id != 3) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav3_x, uavs_msg.uav3_y, 0.0));
    }
    if (uav_id != 4) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav4_x, uavs_msg.uav4_y, 0.0));
    }
    if (uav_id != 5) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav5_x, uavs_msg.uav5_y, 0.0));
    }
    if (uav_id != 6) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav6_x, uavs_msg.uav6_y, 0.0));
    }
    if (uav_id != 7) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav7_x, uavs_msg.uav7_y, 0.0));
    }
    if (uav_id != 8) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav8_x, uavs_msg.uav8_y, 0.0));
    }
    if (uav_id != 9) {
        cloud->points.push_back(pcl::PointXYZ(uavs_msg.uav9_x, uavs_msg.uav9_y, 0.0));
    }
    cloud->width = cloud->points.size();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    uavsPose_pub.publish(cloud_msg);

    // 发送视频流控制数据
    std_msgs::UInt8 video_control_msg;
    video_control_msg.data = video_control_flag;
    video_control_pub.publish(video_control_msg);
}
// -------------------------- Function end ------------------------------

// -------------------------- CallBack start ----------------------------
void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;  // 更新 current_state 的值
    std::string current_mode = msg->mode;

    SendData_array[2] = static_cast<uint8_t>(msg->armed);  // 将无人机是否被武装的状态存储在发送数组的第三个元素
    SendData_array[3] = static_cast<uint8_t>(msg->manual_input);  // 将无人机是否接收手动输入的状态存储在发送数组的第四个元素
    if (current_mode == MODE_PX4_MANUAL) {
        SendData_array[4]=0;
    } else if (current_mode == MODE_PX4_ALTITUDE) {
        SendData_array[4]=3;
    } else if (current_mode == MODE_PX4_POSITION) {
       SendData_array[4]=1;
    } else if (current_mode == MODE_PX4_OFFBOARD) {
        SendData_array[4]=2;
    } else if (current_mode == MODE_PX4_STABILIZED) {
        SendData_array[4]=4;
    } 

    SendData_array[58] = static_cast<uint8_t>(msg->connected);  // 将串口连接状态存储在发送数组的第五十九个元素
}

//电池电压回调函数
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    RosFloatData_array[12] = msg->voltage;  // 将电池电量存储在数组的第13个元素
}

//local_position 位置数据回调函数 
void localpositionposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*
        ENU坐标系 三轴朝向：前左上：东北天
    */
    RosFloatData_array[3] = static_cast<float>(msg->pose.position.x);  // 将无人机位置的x坐标存储在数组的第四个元素
    RosFloatData_array[4] = static_cast<float>(msg->pose.position.y);  // 将无人机位置的y坐标存储在数组的第五个元素
    RosFloatData_array[5] = static_cast<float>(msg->pose.position.z);  // 将无人机位置的z坐标存储在数组的第六个元素

    // 从四元数转换为RPY角（滚转角、俯仰角、偏航角）
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RosFloatData_array[9] = static_cast<float>(roll);  // 将滚转角存储在数组的第十个元素
    RosFloatData_array[10] = static_cast<float>(pitch); // 将俯仰角存储在数组的第十一个元素
    RosFloatData_array[11] = static_cast<float>(yaw);  // 将无人机偏航角存储在数组的第十二个元素
}

//global_position GPS数据
void globalposeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    /*
        GPS数据
    */
    RosFloatData_array[0] = msg->latitude;  // 将纬度存储在数组的第一个元素
    RosFloatData_array[1] = msg->longitude;  // 将经度存储在数组的第二个元素
    RosFloatData_array[2] = msg->altitude;  // 将高度存储在数组的第三个元素
    SendData_array[5] = static_cast<uint8_t>(msg->status.status);  // 确保进行类型转换 定位信息融合状态
    SendData_array[5] = SendData_array[5]+1 ;
}

//local_position速度数据回调函数
void localpositionvelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    RosFloatData_array[6] = msg->twist.linear.x;  // 将无人机线速度的x分量存储在数组的第七个元素
    RosFloatData_array[7] = msg->twist.linear.y;  // 将无人机线速度的y分量存储在数组的第八个元素
    RosFloatData_array[8] = msg->twist.linear.z;  // 将无人机线速度的z分量存储在数组的第九个元素
}
// -------------------------- CallBack end ------------------------------

// -------------------------- Tcp start ---------------------------------
void tcpTimerCallback(const ros::TimerEvent& event) {
    static int callback_counter = 0;
    callback_counter++;
    
    // 每200次(4秒)输出一次基本状态，避免日志刷屏
    if (callback_counter % 200 == 0) {
        ROS_INFO("TCP Timer running - callback #%d, connected: %s, queue_size: %zu", 
                 callback_counter, tcp_server->isConnected() ? "YES" : "NO", 
                 tcp_server->getReceiveQueueSize());
    }
    
    // 检查连接状态
    if (!tcp_server->isConnected()) {
        // ROS_INFO("tcp has not connected");
        return; // 没有连接就直接返回，不阻塞
    }
    
    // === 1. 非阻塞接收处理 ===
    std::vector<uint8_t> receive_buffer;
    while (tcp_server->getReceivedData(receive_buffer)) {  // 处理队列中所有数据
        ROS_INFO("Received %zu bytes", receive_buffer.size());

        // 解析接收到的数据
        if (receive_buffer.size() == 61 && 
            receive_buffer[0] == FRAME_HEADER && 
            receive_buffer[60] == FRAME_TAIL) {
            parseControlCommand(receive_buffer);
        } else {
            ROS_WARN("Received an invalid data frame; skipping this frame.");
        }
    }
    
    // === 2. 数据打包(总是执行) ===
    packSendData();
    
    // === 3. 发送数据 ===
    std::vector<uint8_t> send_data(SendData_array, SendData_array + 60);
    bool send_success = tcp_server->sendBuffer(send_data);
    
    // 只在发送失败时输出警告
    if (!send_success) {
        ROS_WARN("Failed to send UAV data at callback #%d", callback_counter);
    }
    
    // 每50次(1秒)输出一次详细状态

    // ROS_INFO("UAV Status - ID:%d Armed:%d Mode:%d GPS:%d Battery:%.1fV Pos:[%.2f,%.2f,%.2f]", 
    //             SendData_array[1], SendData_array[2], SendData_array[4], SendData_array[5],
    //             RosFloatData_array[12], RosFloatData_array[3], RosFloatData_array[4], RosFloatData_array[5]);

}
// -------------------------- Tcp end -----------------------------------

int main(int argc, char** argv) 
{
    // 初始化SendData_array
    memset(SendData_array, 0, sizeof(SendData_array));
    memset(RosFloatData_array, 0, sizeof(RosFloatData_array));
    
    SendData_array[0] = 0xAA;  // 帧头
    SendData_array[1] = uav_id;  // 无人机ID
    SendData_array[59] = 0xFF;  // 帧尾

    ros::init(argc, argv, "px4_data_reader");
    ros::NodeHandle nh;

    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "192.168.42.62");
    nh.param<int>("port", port, 10000);
    nh.param<int>("uav_id", uav_id, 0);
    tcp_server = new TcpServer(ip_address, port); 

    // 订阅者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, globalposeCallback); 
    ros::Subscriber local_position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, localpositionposeCallback);
    ros::Subscriber local_position_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, localpositionvelocityCallback);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, batteryCallback);

    // 发布者
    ControlCommand_pub = nh.advertise<mavros_msgs::PositionTarget>("/control_data/data", 1);
    taskStatus_pub= nh.advertise<std_msgs::UInt8>("/control_data/task_status", 1);

    uavsPose_pub = nh.advertise<sensor_msgs::PointCloud2>("/control_data/uavs_pose", 1);
    
    video_control_pub = nh.advertise<std_msgs::UInt8>("/control_data/video_control", 1);
    
    ROS_INFO("ROS subscriber successful");

    // 启动TCP服务器
    if (!tcp_server->startServer()) {
        ROS_ERROR("Failed to start TCP server");
        return -1;
    }
    ROS_INFO("TCP Server started on 192.168.3.26:10000");

    // TCP定时器，50ms执行一次 (20Hz)
    ros::Timer tcp_timer = nh.createTimer(ros::Duration(0.05), tcpTimerCallback);

    ROS_INFO("System initialized, spinning...");
    ros::spin();

    return 0;
}