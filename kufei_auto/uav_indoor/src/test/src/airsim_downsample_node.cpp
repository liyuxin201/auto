#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

class TopicDownSampler
{
public:
    TopicDownSampler(ros::NodeHandle& nh)
    {
        nh.param("lidar_rate", lidar_rate_, 20.0);
        nh.param("imu_rate", imu_rate_, 100.0);

        lidar_sub_ = nh.subscribe("/airsim_node/PX4/lidar/LidarSensor1", 10, &TopicDownSampler::lidarCallback, this);
        imu_sub_ = nh.subscribe("/airsim_node/PX4/imu/Imu", 50, &TopicDownSampler::imuCallback, this);

        lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_downsampled", 10);
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu_downsampled", 50);

        last_lidar_pub_time_ = ros::Time(0);
        last_imu_pub_time_ = ros::Time(0);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if ((msg->header.stamp - last_lidar_pub_time_).toSec() >= (1.0 / lidar_rate_)) {
            lidar_pub_.publish(msg);
            last_lidar_pub_time_ = msg->header.stamp;
            std::cout<<"size:"<< (msg->width * msg->height)<<std::endl;
        }
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        if ((msg->header.stamp - last_imu_pub_time_).toSec() >= (1.0 / imu_rate_)) {
            imu_pub_.publish(msg);
            last_imu_pub_time_ = msg->header.stamp;
        }
    }

private:
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher lidar_pub_;
    ros::Publisher imu_pub_;
    double lidar_rate_, imu_rate_;
    ros::Time last_lidar_pub_time_, last_imu_pub_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsim_downsample_node");
    ros::NodeHandle nh("~");
    TopicDownSampler node(nh);
    ros::spin();
    return 0;
}