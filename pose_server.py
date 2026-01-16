#!/usr/bin/env python3
import socket
import threading
import json
import rospy
from geometry_msgs.msg import PoseStamped

class MultiTopicServer:
    def __init__(self, port=21118):
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.running = True

        # Initialize ROS Node
        rospy.init_node('tcp_pose_multiplexer', anonymous=True)
        
        # Subscribe to BOTH topics
        # 1. The estimated (lag compensated) pose
        rospy.Subscriber(
            "/estimated/camera_pose", 
            PoseStamped, 
            lambda msg: self.process_msg(msg, "estimated")
        )
        
        # 2. The raw ORB_SLAM3 pose (useful for scale calculation)
        rospy.Subscriber(
            "/orb_slam3/camera_pose", 
            PoseStamped, 
            lambda msg: self.process_msg(msg, "raw")
        )
        
        # Setup TCP Server
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', self.port))
        self.server_sock.listen(5)

        rospy.loginfo(f"TCP Multiplexer listening on 0.0.0.0:{self.port}")

        # Start thread to accept connections
        self.accept_thread = threading.Thread(target=self.accept_clients)
        self.accept_thread.start()

    def accept_clients(self):
        while self.running and not rospy.is_shutdown():
            try:
                self.server_sock.settimeout(1.0)
                client, addr = self.server_sock.accept()
                rospy.loginfo(f"Client connected: {addr}")
                with self.lock:
                    self.clients.append(client)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Socket error: {e}")

    def process_msg(self, msg, label):
        """
        Common handler for both topics. 
        'label' distinguishes between 'estimated' and 'raw'.
        """
        data = {
            "topic": label,  # <--- Identifier key
            "ts": msg.header.stamp.to_sec(),
            "frame": msg.header.frame_id,
            "p": [ # Compact position [x, y, z]
                round(msg.pose.position.x, 5),
                round(msg.pose.position.y, 5),
                round(msg.pose.position.z, 5)
            ],
            "q": [ # Compact quaternion [x, y, z, w]
                round(msg.pose.orientation.x, 5),
                round(msg.pose.orientation.y, 5),
                round(msg.pose.orientation.z, 5),
                round(msg.pose.orientation.w, 5)
            ]
        }

        self.broadcast(data)

    def broadcast(self, data_dict):
        # Serialize to JSON + Newline
        payload = json.dumps(data_dict) + "\n"
        encoded = payload.encode('utf-8')

        with self.lock:
            disconnected = []
            for client in self.clients:
                try:
                    client.sendall(encoded)
                except (BrokenPipeError, ConnectionResetError):
                    disconnected.append(client)
            
            for d in disconnected:
                self.clients.remove(d)
                d.close()

    def run(self):
        rospy.spin()
        self.running = False
        self.server_sock.close()

if __name__ == '__main__':
    server = MultiTopicServer(port=21118)
    try:
        server.run()
    except rospy.ROSInterruptException:
        pass
