#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <mutex>

class DownsampleNode {
public:
    DownsampleNode() {
        nh_.param<std::string>("pointcloud_topic", pointcloud_topic_name, "/cloud_effected");
        sub_ = nh_.subscribe(pointcloud_topic_name, 1, &DownsampleNode::cloudCallback, this);
        // pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/world_cloud", 1);
        buffer_capacity_ = 3; // 可调整缓冲最大帧数
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // 1. 转成PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        ROS_INFO("latest cloud size: %zu", cloud->points.size());

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.push_back(cloud);
        // 2. 管理缓冲区大小
        while (buffer_.size() > buffer_capacity_) {
            buffer_.pop_front();
        }
        ROS_INFO("buffer size: %zu", buffer_.size());

        // 3. 合并所有缓冲帧
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &ptr : buffer_) {
            *merged += *ptr;
        }

        // 4. 降采样
        pcl::PointCloud<pcl::PointXYZ> downsampled;
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(merged);
        voxel.setLeafSize(0.1f, 0.1f, 0.1f); // 降采样分辨率，可调整
        voxel.filter(downsampled);
        ROS_INFO("downsample cloud size: %zu", downsampled.points.size());

        // 5. 转成ROS消息并发布
        sensor_msgs::PointCloud2 cloud_out;
        pcl::toROSMsg(downsampled, cloud_out);
        cloud_out.header.frame_id = "world"; // 或者"odom"
        cloud_out.header = msg->header;
        pub_.publish(cloud_out);
    }

private:
    std::string pointcloud_topic_name;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer_;
    size_t buffer_capacity_;
    std::mutex buffer_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "downsample_node");
    DownsampleNode node;
    ros::spin();
    return 0;
}