#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import CameraInfo

def main():
    # Initialize the node
    rospy.init_node('camera_info_publisher', anonymous=True)
    
    # Create the publisher
    # Topic: /camera/camera_info
    # Queue size: 10
    pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    
    # Set the publishing rate (e.g., 30 Hz)
    rate = rospy.Rate(30)
    
    # Create the CameraInfo message object
    cam_info = CameraInfo()
    
    # --- Fill in the Data based on your request ---
    
    # 1. Resolution: [640, 480]
    cam_info.width = 640
    cam_info.height = 480
    
    # 2. Distortion Model: radtan -> 'plumb_bob' in standard ROS
    # 'radtan' usually corresponds to 'plumb_bob' (Radial-Tangential)
    cam_info.distortion_model = "plumb_bob"
    
    # 3. Distortion Coeffs: [k1, k2, p1, p2, k3]
    # Input: [0.148509, -0.255395, 0.003505, 0.001639, 0.0]
    cam_info.D = [0.148509, -0.255395, 0.003505, 0.001639, 0.0]
    
    # 4. Intrinsics (K Matrix)
    # Input: [fx, fy, cx, cy] -> [503.640273, 502.167721, 312.565456, 244.436855]
    fx = 503.640273
    fy = 502.167721
    cx = 312.565456
    cy = 244.436855
    
    # The K matrix is 3x3 (flattened to size 9)
    # [fx  0 cx]
    # [ 0 fy cy]
    # [ 0  0  1]
    cam_info.K = [fx, 0.0, cx, 
                  0.0, fy, cy, 
                  0.0, 0.0, 1.0]
    
    # 5. Rectification Matrix (R)
    # Usually Identity for monocular non-rectified cameras
    cam_info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
                  
    # 6. Projection Matrix (P)
    # For a simple monocular setup without stereo rectification, P is often K with an added zero column.
    # [fx  0 cx T_x]
    # [ 0 fy cy T_y]
    # [ 0  0  1 0  ]
    cam_info.P = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]

    # Header frame_id (Coordinate frame of the camera)
    # This should match the frame_id of your image messages
    cam_info.header.frame_id = "camera"

    rospy.loginfo("Publishing CameraInfo to /camera/camera_info")

    while not rospy.is_shutdown():
        # Update timestamp to current time
        cam_info.header.stamp = rospy.Time.now()
        
        # Publish the message
        pub.publish(cam_info)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass