#!/usr/bin/env python2
import rospy
import socket
import threading
import struct
import string

# 导入ROS消息和服务类型
from ubt_msgs.msg import angles_set
from ubt_msgs.srv import servo_read, servo_readRequest

class JointAngleTCPServer:
    def __init__(self, port=21120):
        # 1. 初始化ROS节点
        rospy.init_node('joint_angle_tcp_server', anonymous=True)
        self.lock = threading.Lock()
        
        # 2. 初始化ROS话题发布者和服务代理
        rospy.loginfo("=== [Server Init] Initializing ROS publishers and services... ===")
        # 发布/hal_angles_set话题
        self.angle_pub = rospy.Publisher('/hal_angles_set', angles_set, queue_size=10)
        # 等待hal_servo_read服务就绪
        rospy.wait_for_service('hal_servo_read')
        self.servo_read_srv = rospy.ServiceProxy('hal_servo_read', servo_read)
        rospy.loginfo("=== [Server Init] ROS components are ready. ===")
        
        # 3. 初始化TCP服务器
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', port))
        self.server.listen(5)
        rospy.loginfo("=== [Server Init] Joint angle server starting on 0.0.0.0:{} ===".format(port))
        rospy.loginfo("=== [Server Init] Waiting for client connection... ===")
        
        # 4. 启动请求处理线程
        threading.Thread(target=self.handle_requests, name="RequestHandlerThread").start()
        rospy.spin()
    
    def convert_angle_to_ticks(self, angle_in_degrees):
        """
        实现角度到Ticks的转换（对应C#的ConvertAngleToTicks方法）
        公式：角度 * 2048 / 180
        """
        try:
            return int(float(angle_in_degrees) * 2048.0 / 180.0)
        except (ValueError, TypeError) as e:
            rospy.logerr("=== [Angle Convert] Failed to convert angle: %s ===", e)
            return 0
    
    def publish_joint_angles(self, angle_list, time_ms=100):
        """
        发布关节角度到/hal_angles_set话题（对应C#的PublishJointAngles方法）
        :param angle_list: 长度为17的角度列表（单位：度）
        :param time_ms: 时间参数（int32）
        :return: 发布是否成功
        """
        if len(angle_list) != 17:
            rospy.logerr("=== [Topic Publish] Invalid angle list length: {} (expected 17) ===".format(len(angle_list)))
            return False
        
        try:
            # 1. 转换角度到Ticks
            all_ticks = [self.convert_angle_to_ticks(angle) for angle in angle_list]
            rospy.loginfo("=== [Topic Publish] Converted ticks array: [{}] ===".format(", ".join(map(str, all_ticks))))
            
            # 2. 构建ROS消息并发布
            angle_msg = angles_set()
            angle_msg.angles = all_ticks
            angle_msg.time = int(time_ms)
            
            self.angle_pub.publish(angle_msg)
            rospy.loginfo("=== [Topic Publish] Successfully published to /hal_angles_set (time: {} ms) ===".format(time_ms))
            return True
        
        except Exception as e:
            rospy.logerr("=== [Topic Publish] Failed to publish angle topic: %s ===", e)
            return False
    
    def parse_servo_buffer(self, servo_buf):
        """
        解析servo_read服务返回的buf（形如003A003B...），转换为长度17的角度数组
        步骤：1. 4个字符一组拆分 2. 十六进制转十进制 3. 返回17个元素的列表
        """
        angle_array = []
        
        # 1. 校验buf格式（长度应为17*4=68，确保是4的倍数且能拆分为17组）
        if len(servo_buf) % 4 != 0:
            rospy.logerr("=== [Buffer Parse] Invalid buf length: {} (not a multiple of 4) ===".format(len(servo_buf)))
            return [0]*17
        
        # 2. 4个字符一组拆分
        for i in range(0, min(len(servo_buf), 68), 4):
            hex_group = servo_buf[i:i+4]
            # 3. 十六进制转十进制
            try:
                dec_value = int(hex_group, 16)
                angle_array.append(dec_value)
            except ValueError as e:
                rospy.logwarn("=== [Buffer Parse] Failed to convert hex group {}: %s ===".format(hex_group, e))
                angle_array.append(0)
        
        # 4. 确保返回长度为17（不足补0，超出截断）
        if len(angle_array) < 17:
            angle_array += [0] * (17 - len(angle_array))
        elif len(angle_array) > 17:
            angle_array = angle_array[:17]
        
        rospy.loginfo("=== [Buffer Parse] Parsed angle array: [{}] ===".format(", ".join(map(str, angle_array))))
        return angle_array
    
    def get_joint_angles(self, req_type=0, req_buf=""):
        """
        调用hal_servo_read服务，获取并解析关节角度
        :param req_type: 服务请求type参数（int32）
        :param req_buf: 服务请求buf参数（string）
        :return: 长度为17的角度数组
        """
        rospy.loginfo("=== [Service Call] Start calling hal_servo_read (type: {}, buf: {}) ===".format(req_type, req_buf))
        
        try:
            # 1. 构建服务请求
            srv_request = servo_readRequest()
            srv_request.type = int(req_type)
            srv_request.buf = req_buf
            
            # 2. 调用ROS服务
            start_time = rospy.get_time()
            response = self.servo_read_srv(srv_request)
            cost_time = (rospy.get_time() - start_time) * 1000
            rospy.loginfo("=== [Service Call] Call success, cost {:.2f} ms (rc: {}) ===".format(cost_time, response.rc))
            
            # 3. 解析返回的buf，获取角度数组
            return self.parse_servo_buffer(response.servo_angles)
        
        except rospy.ServiceException as e:
            rospy.logerr("=== [Service Call] Failed to call hal_servo_read: %s ===", e)
            return [0]*17
    
    def handle_client_request(self, request_data):
        """
        处理客户端请求数据，区分设置角度和获取角度指令
        协议约定（简单文本协议，便于调试）：
        - 设置角度：SET <time_ms> <angle1> <angle2> ... <angle17>
        - 获取角度：GET <type> <buf>
        :param request_data: 客户端发送的请求数据（字符串）
        :return: 响应数据（字符串，包含执行结果或角度数组）
        """
        try:
            request_lines = request_data.strip().splitlines()
            if not request_lines:
                return "ERROR: Empty request"
            
            cmd = request_lines[0].split()[0].upper()
            
            # 1. 处理设置角度指令
            if cmd == "SET":
                parts = request_lines[0].split()
                if len(parts) != 19:  # SET + time_ms + 17个角度
                    return "ERROR: SET command format error (expected: SET <time> <angle1>...<angle17>)"
                
                time_ms = parts[1]
                angle_list = parts[2:19]
                
                # 执行发布操作
                success = self.publish_joint_angles(angle_list, time_ms)
                return "SUCCESS: Angle published" if success else "ERROR: Failed to publish angle"
            
            # 2. 处理获取角度指令
            elif cmd == "GET":
                parts = request_lines[0].split(maxsplit=2)
                if len(parts) < 2:
                    return "ERROR: GET command format error (expected: GET <type> [buf])"
                
                req_type = parts[1]
                req_buf = parts[2] if len(parts) == 3 else ""
                
                # 执行服务调用并解析角度
                angle_array = self.get_joint_angles(req_type, req_buf)
                return "SUCCESS: " + ",".join(map(str, angle_array))
            
            # 3. 未知指令
            else:
                return "ERROR: Unknown command (supported: SET, GET)"
        
        except Exception as e:
            rospy.logerr("=== [Request Handle] Failed to process client request: %s ===", e)
            return "ERROR: " + str(e)
    
    def handle_client(self, conn, addr):
        """处理单个客户端连接，参考server_reference的客户端处理逻辑"""
        try:
            # 设置接收超时（5秒），避免无限阻塞
            conn.settimeout(5.0)
            rospy.loginfo("=== [Connection] New client connected: {}:{} ===".format(addr[0], addr[1]))
            
            # 接收客户端请求数据
            request = conn.recv(4096).decode('utf-8', errors='ignore')
            if not request:
                rospy.logwarn("=== [Connection] No valid request received from {}:{} ===".format(addr[0], addr[1]))
                conn.sendall("ERROR: Empty request".encode('utf-8'))
                return
            
            rospy.loginfo("=== [Connection] Received request from {}:{} (size: {} bytes):\n{} ===".format(
                addr[0], addr[1], len(request), request[:200] + "..." if len(request) > 200 else request
            ))
            
            # 处理请求并获取响应
            response = self.handle_client_request(request)
            
            # 发送响应数据给客户端
            conn.sendall(response.encode('utf-8'))
            rospy.loginfo("=== [Connection] Response sent to {}:{}: {}".format(addr[0], addr[1], response[:100]))
        
        except socket.timeout:
            rospy.logerr("=== [Connection] Recv timeout (5s) from {}:{} ===".format(addr[0], addr[1]))
            conn.sendall("ERROR: Timeout".encode('utf-8'))
        except Exception as e:
            rospy.logerr("=== [Connection] Failed to handle client {}:{}: %s ===".format(addr[0], addr[1], e))
            conn.sendall(("ERROR: " + str(e)).encode('utf-8'))
        finally:
            conn.close()
            rospy.loginfo("=== [Connection] Connection closed with {}:{} ===".format(addr[0], addr[1]))
    
    def handle_requests(self):
        """循环接受客户端连接，启动独立线程处理每个客户端"""
        while not rospy.is_shutdown():
            try:
                conn, addr = self.server.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr), name="ClientHandler-{}".format(addr[0])).start()
            except Exception as e:
                rospy.logerr("=== [Connection] Failed to accept connection: %s ===", e)
                break
        rospy.loginfo("=== [Connection] Request handler thread exited ===")

if __name__ == '__main__':
    try:
        JointAngleTCPServer(21120)
    except rospy.ROSInterruptException:
        rospy.loginfo("=== [Server] Node interrupted, exiting ===")
    except Exception as e:
        rospy.logerr("=== [Server] Fatal error: %s ===", e)
