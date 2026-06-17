// #include "px4/px4_control.h"

// //实例化一个无人机控制器

// int main(int argc,char *argv[])
// {
//     ros::init(argc,argv,"px4_control");
//     ros::NodeHandle nh;
    
//     ROS_INFO("OK");
//     px4::control px4_control;
//     while(! px4_control.init(nh));
//     while(! px4_control.start());
    
    
//     ros::Rate rate(10);
//     int i = 0;
//     ROS_INFO("while");
    
//     while(ros::ok)
//     {
//         ROS_INFO("while");
//         // ros::spinOnce();
//         rate.sleep();
//     }

// }