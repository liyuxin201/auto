#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>

/*fastlio版本 去除坐标转换，只做滤波*/

Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;

// 新增：前左上到ENU坐标系转换函数
Eigen::Vector3d transform_FLU_to_ENU_position(const Eigen::Vector3d& p_flu) {
    Eigen::Vector3d p_enu_converted;
    // 前左上 -> ENU转换
    // FLU: X=前，Y=左，Z=上
    // ENU: X=东，Y=北，Z=上
    p_enu_converted.x() = -p_flu.y();  // ENU东 = -FLU左
    p_enu_converted.y() = p_flu.x();   // ENU北 = FLU前
    p_enu_converted.z() = p_flu.z();   // ENU上 = FLU上
    return p_enu_converted;
}

// 新增：前左上到ENU四元数转换函数
Eigen::Quaterniond transform_FLU_to_ENU_quaternion(const Eigen::Quaterniond& q_flu) {
    // 前左上到ENU的转换矩阵（绕Z轴顺时针90度）
    Eigen::Matrix3d R_flu_to_enu;
    R_flu_to_enu << 0, -1,  0,
                    1,  0,  0,
                    0,  0,  1;
    
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d R_flu = q_flu.toRotationMatrix();
    
    // 应用坐标系转换
    Eigen::Matrix3d R_enu = R_flu_to_enu * R_flu;
    
    // 转换回四元数
    Eigen::Quaterniond q_enu_converted(R_enu);
    q_enu_converted.normalize();
    
    return q_enu_converted;
}

class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

    double addData(double newData) {
        if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        // 返回当前窗口内的平均值
        return windowAvg;
    }

    int get_size(){
        return dataQueue.size();
    }

    double get_avg(){
        return windowAvg;
    }

private:
    int windowSize;
    double windowSum;
    double windowAvg;
    std::queue<double> dataQueue;
};

int windowSize = 8;
SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}


void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 新增：接收前左上坐标系数据
    Eigen::Vector3d p_flu_received(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q_flu_received(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 新增：转换到ENU坐标系后赋值给原有变量
    // p_lidar_body = transform_FLU_to_ENU_position(p_flu_received);
    // q_mav = transform_FLU_to_ENU_quaternion(q_flu_received);
    p_lidar_body = p_flu_received;
    q_mav = q_flu_received;
    
    // 新增：调试输出（可选）
    static int debug_counter = 0;
    if (debug_counter++ % 100 == 0) {  // 每100次输出一次
        ROS_INFO("Coordinate Transform: FLU[%.3f,%.3f,%.3f] -> ENU[%.3f,%.3f,%.3f]", 
                 p_flu_received.x(), p_flu_received.y(), p_flu_received.z(),
                 p_lidar_body.x(), p_lidar_body.y(), p_lidar_body.z());
    }
}
 
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("window start");
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    std::string odometry_topic_name;
    nh.param<std::string>("odometry_topic", odometry_topic_name, "/Odometry");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>(odometry_topic_name, 100, vins_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    ros::Time last_request = ros::Time::now();
    float init_yaw = 0.0;
    bool init_flag = 1;
    Eigen::Quaterniond init_q;
    while(ros::ok()){
        ROS_INFO("fast_lio start");

        if(swa.get_size()==windowSize&&!init_flag){
            init_yaw = swa.get_avg();
            init_flag = 1;
            init_q = Eigen::AngleAxisd(init_yaw,Eigen::Vector3d::UnitZ())//des.yaw
    * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX());
        // delete swa;
        }

        if(init_flag){
            geometry_msgs::PoseStamped vision;
            // p_enu = init_q*p_lidar_body;
            p_enu = p_lidar_body;
    
            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];
    
            vision.pose.orientation.x = q_mav.x();
            vision.pose.orientation.y = q_mav.y();
            vision.pose.orientation.z = q_mav.z();
            vision.pose.orientation.w = q_mav.w();
    
            vision.header.stamp = ros::Time::now();
            vision_pub.publish(vision);
    
            ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
            p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());

        }

 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
