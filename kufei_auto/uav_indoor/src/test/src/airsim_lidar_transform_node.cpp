#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <mutex>

class LidarWorldTransform
{
public:
    LidarWorldTransform(ros::NodeHandle& nh)
    {
        odom_sub_ = nh.subscribe("/airsim_node/PX4/odom_local_ned", 1, &LidarWorldTransform::odomCB, this);
        lidar_sub_ = nh.subscribe("/airsim_node/PX4/lidar/LidarSensor1", 1, &LidarWorldTransform::lidarCB, this);
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_in_world", 1);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_in_world", 1);
        pose_recved_ = false;
    }

private:
    ros::Subscriber odom_sub_, lidar_sub_;
    ros::Publisher cloud_pub_, odom_pub_;


    Eigen::Quaterniond pose_q_;
    Eigen::Vector3d pose_t_;
    std::mutex mtx_;
    bool pose_recved_;

    void odomCB(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        pose_q_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        pose_t_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        pose_recved_ = true;
        nav_msgs::Odometry msgout=*msg;
        msgout.header.frame_id = "world"; // 或者"odom"，按你坐标系定义
        odom_pub_.publish(msgout);
    }

    void lidarCB(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if (!pose_recved_) return;

        std::cout<<"size:"<< (msg->width * msg->height)<<std::endl;

        pcl::PointCloud<pcl::PointXYZI> cloud_in;
        pcl::fromROSMsg(*msg, cloud_in);


        // 构造 Eigen 4x4 变换矩阵（机体->世界）
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            q = pose_q_;
            t = pose_t_;
        }
        transform.block<3,3>(0,0) = q.toRotationMatrix();
        transform.block<3,1>(0,3) = t;
        Eigen::Matrix4f transform_f = transform.cast<float>();

        // 点云变换
        pcl::PointCloud<pcl::PointXYZI> cloud_out;
        pcl::transformPointCloud(cloud_in, cloud_out, transform_f);

        // 发布
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_out, cloud_msg);
        cloud_msg.header.stamp = msg->header.stamp;
        cloud_msg.header.frame_id = "world_ned"; // 或者"odom"，按你坐标系定义
        cloud_pub_.publish(cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsim_lidar_transform_node");
    ros::NodeHandle nh;
    LidarWorldTransform node(nh);
    ros::spin();
    return 0;
}