/*发布里程计信息*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <mutex>


ros::Subscriber odom_sub_;
ros::Publisher odom_pub_;

void odomCB(const nav_msgs::OdometryConstPtr& msg)
    {
        std::cout<<"recieve odom"<<std::endl;
        nav_msgs::Odometry msgout=*msg;
        msgout.header.frame_id = "world"; // 或者"odom"，按你坐标系定义
        odom_pub_.publish(msgout);
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_livo_odom_node");
    ros::NodeHandle nh;

    std::string odometry_topic_name;
    nh.param<std::string>("odometry_topic", odometry_topic_name, "/Odometry");

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_in_world", 1);
    // odom_sub_ = nh.subscribe("/aft_mapped_to_init", 1, odomCB);
    odom_sub_ = nh.subscribe(odometry_topic_name, 1, odomCB);
    // odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, odomCB);


    ros::spin();
    return 0;
}