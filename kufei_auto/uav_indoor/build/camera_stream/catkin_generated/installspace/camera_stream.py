#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError

class CameraStreamNode:
    def __init__(self):
        rospy.init_node('camera_stream', anonymous=True)
        
        # 从ROS参数服务器获取参数
        self.server_ip = rospy.get_param('~server_ip')
        self.server_port = rospy.get_param('~server_port')
        self.image_topic = rospy.get_param('~image_topic')
        self.control_topic = rospy.get_param('~control_topic', '/control_data/video_control')
        self.jpeg_quality = rospy.get_param('~jpeg_quality')
        self.frame_rate = rospy.get_param('~frame_rate')
        self.output_width = rospy.get_param('~output_width')
        self.output_height = rospy.get_param('~output_height')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_addr = (self.server_ip, self.server_port)
        
        # 推流控制标志
        self.stream_enabled = False
        
        # 帧率控制
        self.rate = rospy.Rate(self.frame_rate)
        self.last_send_time = rospy.Time.now()
        
        # 统计信息
        self.frame_count = 0
        self.start_time = rospy.Time.now()
        
        # 订阅图像话题和控制话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.control_sub = rospy.Subscriber(self.control_topic, UInt8, self.control_callback)
        
        rospy.loginfo(f"相机推流节点已启动，服务端: {self.server_ip}:{self.server_port}")
        rospy.loginfo(f"订阅图像话题: {self.image_topic}")
        rospy.loginfo(f"订阅控制话题: {self.control_topic}")
        rospy.loginfo(f"输出分辨率: {self.output_width}x{self.output_height}")
        rospy.loginfo(f"推流帧率: {self.frame_rate} fps")
        rospy.loginfo(f"JPEG质量: {self.jpeg_quality}")
        rospy.loginfo(f"初始状态: 推流{'已启用' if self.stream_enabled else '未启用'}")
        
    def control_callback(self, msg):
        """控制话题回调函数"""
        if msg.data == 1:
            if not self.stream_enabled:
                self.stream_enabled = True
                self.frame_count = 0
                self.start_time = rospy.Time.now()
                rospy.loginfo("收到启动命令，开始推流")
        else:
            if self.stream_enabled:
                self.stream_enabled = False
                rospy.loginfo("收到停止命令，停止推流")
                
    def resize_image(self, image):
        """缩放图像到指定尺寸"""
        if image.shape[1] == self.output_width and image.shape[0] == self.output_height:
            return image
            
        return cv2.resize(image, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)
    
    def image_callback(self, msg):
        # 检查是否启用推流
        if not self.stream_enabled:
            return
            
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 控制帧率
            current_time = rospy.Time.now()
            if (current_time - self.last_send_time).to_sec() < 1.0 / self.frame_rate:
                return
            self.last_send_time = current_time
            
            # 缩放图像到指定尺寸
            resized_image = self.resize_image(cv_image)
            
            # 压缩图像为JPEG
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, encoded_frame = cv2.imencode('.jpg', resized_image, encode_param)
            
            if not success:
                rospy.logwarn("图像编码失败")
                return
                
            frame_data = encoded_frame.tobytes()
            
            # 发送数据
            try:
                self.sock.sendto(frame_data, self.server_addr)
                self.frame_count += 1
                
                # 每30帧打印一次统计信息
                if self.frame_count % 30 == 0:
                    elapsed = (rospy.Time.now() - self.start_time).to_sec()
                    fps = self.frame_count / elapsed
                    rospy.loginfo(f"已发送 {self.frame_count} 帧, 平均FPS: {fps:.2f}, 数据大小: {len(frame_data)} 字节")
                    
            except socket.error as e:
                rospy.logwarn(f"发送数据失败: {e}")
                
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"处理图像时发生错误: {e}")
    
    def run(self):
        rospy.spin()
        
    def shutdown(self):
        rospy.loginfo("关闭相机推流节点")
        self.sock.close()

if __name__ == '__main__':
    try:
        node = CameraStreamNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass