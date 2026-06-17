#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h> // 根据需要修改
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

ros::Publisher pub; // 根据需要修改

void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::cout<<"size"<<(msg->width * msg->height)<<std::endl;
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);

    output_msg.header.frame_id = "map"; // 修改为所需的frame_id
    pub.publish(output_msg);

    std::cout << "输入任意键" <<std::endl;
    std::string line;
    std::getline(std::cin, line);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsimpc_test_node");
    ros::NodeHandle nh;
    pub=nh.advertise<sensor_msgs::PointCloud2>("airsimPC", 1);
    ros::Subscriber sub=nh.subscribe("/airsim_node/PX4/lidar/LidarSensor1",1,Callback);

    ros::spin(); // 进入循环，等待回调

    return 0;
}
