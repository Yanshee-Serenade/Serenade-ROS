#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CompressedImageToImage:
    def __init__(self):
        rospy.init_node('compressed_to_raw', anonymous=True)
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # --- 计时逻辑初始化 ---
        self.last_slow_time = rospy.Time(0)
        self.slow_interval = rospy.Duration(0, 190000000)  # 5 Hz = 0.2s 间隔
        
        # 订阅压缩图像话题
        self.compressed_sub = rospy.Subscriber(
            '/camera/image_compressed', 
            CompressedImage, 
            self.compressed_callback,
            queue_size=1
        )
        
        # 发布解压后的图像话题 (原始频率)
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            Image, 
            queue_size=1
        )

        # 发布低频图像话题 (5 Hz)
        self.slow_pub = rospy.Publisher(
            '/camera/image_slow', 
            Image, 
            queue_size=1
        )
        
        rospy.loginfo("开始转换压缩图像...")
        rospy.loginfo("发布: /camera/image_raw (原始频率)")
        rospy.loginfo("发布: /camera/image_slow (限制在 5Hz)")
        
    def compressed_callback(self, msg):
        try:
            # 将压缩图像消息转换为OpenCV图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn("无法解码压缩图像")
                return
            
            # 转换为 ROS Image 消息
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 复制时间戳和帧ID
            image_msg.header.stamp = msg.header.stamp
            image_msg.header.frame_id = msg.header.frame_id
            
            # 1. 始终发布到原始原始频率话题
            self.image_pub.publish(image_msg)
            
            # 2. 检查是否达到 5Hz 的时间间隔并发布到 slow 话题
            current_time = rospy.Time.now()
            if (current_time - self.last_slow_time) >= self.slow_interval:
                self.slow_pub.publish(image_msg)
                self.last_slow_time = current_time
            
        except CvBridgeError as e:
            rospy.logerr("CV桥接错误: %s", str(e))
        except Exception as e:
            rospy.logerr("处理图像时出错: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = CompressedImageToImage()
        converter.run()
    except rospy.ROSInterruptException:
        pass
