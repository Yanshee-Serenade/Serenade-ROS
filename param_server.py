#!/usr/bin/env python3
import rospy
import socketserver
import json
import threading

class ParamRequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        # Read the data from the connection (expects a single line of JSON)
        data = self.rfile.readline().strip()
        if not data:
            return

        try:
            # Parse JSON
            request = json.loads(data.decode('utf-8'))
            path = request.get('path')
            value = request.get('value')

            if path is not None:
                # Set the ROS parameter
                rospy.set_param(path, value)
                rospy.loginfo(f"Proxy: Set '{path}' to '{value}'")
            else:
                rospy.logwarn("Proxy: Received request without 'path'")

        except Exception as e:
            rospy.logerr(f"Proxy Error: {e}")

class ParamProxyServer:
    def __init__(self, port=21119):
        self.port = port
        self.server = None

    def start(self):
        rospy.init_node('tcp_param_proxy', anonymous=True)
        
        # Allow address reuse to prevent "Address already in use" on restarts
        socketserver.TCPServer.allow_reuse_address = True
        
        # Create the server
        self.server = socketserver.TCPServer(("", self.port), ParamRequestHandler)
        
        # Run server in a separate thread so ROS callbacks (if any) aren't blocked
        # though strictly not necessary for just set_param, it's good practice.
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        
        rospy.loginfo(f"TCP Param Proxy listening on port {self.port}")
        rospy.spin()

    def shutdown(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()

if __name__ == "__main__":
    try:
        s = ParamProxyServer()
        s.start()
    except rospy.ROSInterruptException:
        pass
