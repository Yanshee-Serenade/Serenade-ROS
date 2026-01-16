#!/usr/bin/env python2
import socket
import struct
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge

# 导入ROS服务相关消息类型
from orb_slam3_ros.srv import GetTrackingData, GetTrackingDataRequest


class ImageTrackingDataServer:
    def __init__(self, port=21121):
        rospy.init_node("tracking_data_byte_server", anonymous=True)
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # 等待ROS服务就绪
        rospy.loginfo(
            "=== [Server Init] Waiting for orb_slam3/get_tracking_data service... ==="
        )
        rospy.wait_for_service("orb_slam3/get_tracking_data")
        self.get_tracking_data_srv = rospy.ServiceProxy(
            "orb_slam3/get_tracking_data", GetTrackingData
        )
        rospy.loginfo("=== [Server Init] Service is ready. ===")

        # 初始化TCP服务器
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(5)
        print(
            "=== [Server Init] Byte stream server starting on 0.0.0.0:{} ===".format(
                port
            )
        )
        print("=== [Server Init] Waiting for client connection... ===")

        # 启动请求处理线程
        threading.Thread(
            target=self.handle_requests, name="RequestHandlerThread"
        ).start()
        rospy.spin()

    def get_ros_tracking_data(self):
        """调用ROS服务，获取更新后的跟踪数据（含双点云），添加详细调试日志"""
        rospy.loginfo(
            "=== [ROS Service] Start calling orb_slam3_ros/get_tracking_data ==="
        )
        try:
            # 调用ROS服务（无请求参数）
            start_time = time.time()
            response = self.get_tracking_data_srv(GetTrackingDataRequest())
            cost_time = (time.time() - start_time) * 1000
            rospy.loginfo(
                "=== [ROS Service] Call success, cost {:.2f} ms ===".format(cost_time)
            )

            # 调试：打印ROS服务返回数据的关键信息
            if response:
                rospy.loginfo("=== [ROS Service] Response info: ===")
                # Note: We are no longer packing intrinsics, but logging them here for debug is still fine
                rospy.loginfo("  - Success: {}".format(response.success))
                rospy.loginfo(
                    "  - Camera pose valid: {}".format(bool(response.camera_pose))
                )
                rospy.loginfo(
                    "  - Camera point cloud data len: {}".format(
                        len(response.tracked_points_camera.data)
                    )
                )
                rospy.loginfo(
                    "  - World point cloud data len: {}".format(
                        len(response.tracked_points_world.data)
                    )
                )
                rospy.loginfo(
                    "  - Image valid: {}".format(not response.current_image.data == "")
                )
            return response

        except rospy.ServiceException as e:
            rospy.logerr("=== [ROS Service] Call failed: %s ===", e)
            return None

    def pack_data_to_bytes(self, tracking_data):
        """将更新后的ROS服务数据封装为自定义字节流，添加详细打包调试日志"""
        rospy.loginfo("=== [Byte Pack] Start packing data to byte stream ===")
        if tracking_data is None:
            rospy.logerr("=== [Byte Pack] Tracking data is None, return empty ===")
            return None

        byte_buffer = []
        total_pack_size = 0

        # 1. 封装服务状态（bool，转换为uint8，1字节）
        success = 1 if tracking_data.success else 0
        success_bytes = struct.pack(">B", success)
        byte_buffer.append(success_bytes)
        total_pack_size += len(success_bytes)
        rospy.loginfo("=== [Byte Pack] 1. Success packed: 1 byte ===")

        # 2. 封装相机位姿（7个float64，共56字节；空位姿填充0）
        pose = tracking_data.camera_pose
        try:
            p = pose.pose.position
            o = pose.pose.orientation
            pose_data = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        except:
            rospy.logwarn("=== [Byte Pack] 2. Pose invalid, fill default value ===")
            pose_data = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)  # 默认单位四元数
        pose_bytes = struct.pack(">ddddddd", *pose_data)
        byte_buffer.append(pose_bytes)
        total_pack_size += len(pose_bytes)
        rospy.loginfo("=== [Byte Pack] 2. Pose packed: 56 bytes ===")

        # 3. 封装相机坐标点云（tracked_points_camera）
        point_cloud_camera = tracking_data.tracked_points_camera
        pc_camera_data = point_cloud_camera.data if point_cloud_camera.data else b""
        pc_camera_len = len(pc_camera_data)
        pc_camera_len_bytes = struct.pack(">I", pc_camera_len)
        byte_buffer.append(pc_camera_len_bytes)
        byte_buffer.append(pc_camera_data)
        total_pack_size += len(pc_camera_len_bytes) + pc_camera_len
        rospy.loginfo(
            "=== [Byte Pack] 3. Camera point cloud packed: {} (len) + {} (data) = {} bytes ===".format(
                len(pc_camera_len_bytes),
                pc_camera_len,
                len(pc_camera_len_bytes) + pc_camera_len,
            )
        )

        # 4. 封装世界坐标点云（tracked_points_world）
        point_cloud_world = tracking_data.tracked_points_world
        pc_world_data = point_cloud_world.data if point_cloud_world.data else b""
        pc_world_len = len(pc_world_data)
        pc_world_len_bytes = struct.pack(">I", pc_world_len)
        byte_buffer.append(pc_world_len_bytes)
        byte_buffer.append(pc_world_data)
        total_pack_size += len(pc_world_len_bytes) + pc_world_len
        rospy.loginfo(
            "=== [Byte Pack] 4. World point cloud packed: {} (len) + {} (data) = {} bytes ===".format(
                len(pc_world_len_bytes),
                pc_world_len,
                len(pc_world_len_bytes) + pc_world_len,
            )
        )

        # 5. 封装图像数据（先jpg长度，后jpg字节数据）
        image = tracking_data.current_image
        img_jpg = b""
        try:
            # 将sensor_msgs/Image转换为cv::Mat，再编码为jpg
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            _, buffer = cv2.imencode(".jpg", cv_image)
            img_jpg = buffer.tobytes()
        except Exception as e:
            rospy.logwarn("=== [Byte Pack] 5. Image encode failed: %s ===", e)
        # 封装图像长度（uint32，4字节）+ jpg数据
        img_len = len(img_jpg)
        img_len_bytes = struct.pack(">I", img_len)
        byte_buffer.append(img_len_bytes)
        byte_buffer.append(img_jpg)
        total_pack_size += len(img_len_bytes) + img_len
        rospy.loginfo(
            "=== [Byte Pack] 5. Image packed: {} (len) + {} (data) = {} bytes ===".format(
                len(img_len_bytes), img_len, len(img_len_bytes) + img_len
            )
        )

        # 拼接所有字节数据，返回完整字节流
        final_byte_stream = b"".join(byte_buffer)
        rospy.loginfo(
            "=== [Byte Pack] Total byte stream size: {} bytes ===".format(
                total_pack_size
            )
        )
        rospy.loginfo("=== [Byte Pack] Packing completed ===")

        return final_byte_stream

    def handle_requests(self):
        """循环接受客户端连接，添加连接调试日志"""
        while not rospy.is_shutdown():
            try:
                conn, addr = self.server.accept()
                rospy.loginfo(
                    "=== [Connection] New client connected: {}:{} ===".format(
                        addr[0], addr[1]
                    )
                )
                threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr),
                    name="ClientHandler-{}".format(addr[0]),
                ).start()
            except Exception as e:
                rospy.logerr("=== [Connection] Accept connection failed: %s ===", e)
                break
        rospy.loginfo("=== [Connection] Request handler thread exited ===")

    def handle_client(self, conn, addr):
        """处理单个客户端请求，解决请求阻塞问题+添加详细调试日志"""
        try:
            # 优化1：设置recv超时（避免无限阻塞在recv(1024)），超时时间5秒
            conn.settimeout(5.0)

            # 优化2：无需等待1024字节，接收任意长度有效请求（解决客户端未发送足够数据的问题）
            rospy.loginfo(
                "=== [Client {}] Waiting for client request... ===".format(addr[0])
            )
            request = conn.recv(1024)  # 改为接收任意长度，而非等待满1024字节

            if not request:
                rospy.logwarn(
                    "=== [Client {}] No valid request received, closing connection ===".format(
                        addr[0]
                    )
                )
                conn.sendall(struct.pack(">I", 0))  # 发送空数据标记
                return

            rospy.loginfo(
                "=== [Client {}] Received request (size: {} bytes): {}".format(
                    addr[0],
                    len(request),
                    request[:50] if len(request) > 50 else request,
                )
            )

            # 1. 获取ROS服务数据（含双点云）
            rospy.loginfo(
                "=== [Client {}] Start fetching ROS service data ===".format(addr[0])
            )
            tracking_data = self.get_ros_tracking_data()

            # 2. 封装为字节流（含双点云）
            rospy.loginfo(
                "=== [Client {}] Start packing byte stream ===".format(addr[0])
            )
            byte_stream = self.pack_data_to_bytes(tracking_data)

            # 3. 发送字节流（无额外协议，纯数据传输）
            if byte_stream:
                rospy.loginfo(
                    "=== [Client {}] Sending byte stream (size: {} bytes) ===".format(
                        addr[0], len(byte_stream)
                    )
                )
                # 优化3：分块发送大数据（避免单次发送过大导致阻塞）
                chunk_size = 4096
                for i in range(0, len(byte_stream), chunk_size):
                    chunk = byte_stream[i : i + chunk_size]
                    conn.sendall(chunk)
                rospy.loginfo(
                    "=== [Client {}] Byte stream sent completely ===".format(addr[0])
                )
            else:
                rospy.logwarn(
                    "=== [Client {}] Empty byte stream, sending null marker ===".format(
                        addr[0]
                    )
                )
                conn.sendall(struct.pack(">I", 0))

        except socket.timeout:
            rospy.logerr(
                "=== [Client {}] Request recv timeout (5s), no data received ===".format(
                    addr[0]
                )
            )
        except Exception as e:
            rospy.logerr("=== [Client {}] Handle failed: %s ===".format(addr[0]), e)
        finally:
            conn.close()
            rospy.loginfo("=== [Client {}] Connection closed ===".format(addr[0]))


if __name__ == "__main__":
    try:
        ImageTrackingDataServer(21121)
    except rospy.ROSInterruptException:
        rospy.loginfo("=== [Server] Node interrupted, exiting ===")
    except Exception as e:
        rospy.logerr("=== [Server] Fatal error: %s ===", e)
