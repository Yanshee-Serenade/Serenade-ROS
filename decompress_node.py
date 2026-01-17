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
        
        # 订阅压缩图像话题
        self.compressed_sub = rospy.Subscriber(
            '/camera/image_compressed', 
            CompressedImage, 
            self.compressed_callback,
            queue_size=1
        )
        
        # 发布解压后的图像话题
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            Image, 
            queue_size=1
        )
        
        rospy.loginfo("开始转换压缩图像为BGR原始图像...")
        rospy.loginfo("订阅: /camera/image_compressed")
        rospy.loginfo("发布: /camera/image_raw")
        
    def compressed_callback(self, msg):
        try:
            # 将压缩图像消息转换为OpenCV图像（直接得到BGR格式）
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn("无法解码压缩图像")
                return
            
            # 关键修改1：移除BGR转RGB的步骤，直接使用OpenCV解码后的BGR图像
            # 关键修改2：cv2_to_imgmsg的编码格式改为"bgr8"（对应OpenCV的BGR格式）
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 复制时间戳和帧ID以保持同步
            image_msg.header.stamp = msg.header.stamp
            image_msg.header.frame_id = msg.header.frame_id
            
            # 发布图像
            self.image_pub.publish(image_msg)
            
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
