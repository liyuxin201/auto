cmake_minimum_required(VERSION 3.0.2)
project(camera_stream)

## 编译系统依赖: find_package
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

## 系统非catkin依赖
find_package(Boost REQUIRED COMPONENTS system)

# 查找OpenCV
find_package(OpenCV REQUIRED)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    sensor_msgs 
    cv_bridge 
    image_transport
  DEPENDS 
    OpenCV
)


## 指定安装的Python脚本
catkin_install_python(PROGRAMS
  src/camera_stream.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)