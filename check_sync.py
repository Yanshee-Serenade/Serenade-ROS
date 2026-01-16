#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped

class SyncMonitor:
    def __init__(self):
        rospy.init_node('sensor_sync_monitor', anonymous=True)
        
        # Latest timestamps (epochs)
        self.ts_imu = None
        self.ts_cam = None
        self.ts_pose = None
        
        # Subscribers
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/camera/image_raw', Image, self.cam_cb)
        rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, self.pose_cb)

        rospy.loginfo("Sync Monitor Started. Waiting for valid ROS time...")

    def imu_cb(self, msg):
        self.ts_imu = msg.header.stamp.to_sec()

    def cam_cb(self, msg):
        self.ts_cam = msg.header.stamp.to_sec()

    def pose_cb(self, msg):
        self.ts_pose = msg.header.stamp.to_sec()

    def run(self):
        rate = rospy.Rate(10)
        
        # 1. Capture Start Times (T0)
        # We wait for a non-zero ROS time in case of simulation/bag delays
        while rospy.Time.now().is_zero() and not rospy.is_shutdown():
            time.sleep(0.1)

        t0_sys = time.time()
        t0_ros = rospy.Time.now().to_sec()

        # 2. Print Header (Total width ~72 chars)
        # Format: 5 cols. Each ~12 chars wide + separators.
        print(f"\nBASE TIMES | Sys: {t0_sys:.2f} | ROS: {t0_ros:.2f}")
        print("=" * 72)
        print(f"{'SYS(rel)':>10} | {'ROS(rel)':>10} | {'IMU(rel)':>10} | {'CAM(rel)':>10} | {'POS(rel)':>10}")
        print("-" * 72)

        while not rospy.is_shutdown():
            # Current Times
            now_sys = time.time()
            now_ros = rospy.Time.now().to_sec()

            # Calculate Deltas (Relative to Script Start)
            d_sys = now_sys - t0_sys
            d_ros = now_ros - t0_ros

            # For topics, we compare them to the ROS start time (t0_ros)
            # so they align visually with the ROS(rel) column.
            if self.ts_imu:
                d_imu_str = f"{self.ts_imu - t0_ros:>10.3f}"
            else:
                d_imu_str = f"{'--':>10}"

            if self.ts_cam:
                d_cam_str = f"{self.ts_cam - t0_ros:>10.3f}"
            else:
                d_cam_str = f"{'--':>10}"

            if self.ts_pose:
                d_pose_str = f"{self.ts_pose - t0_ros:>10.3f}"
            else:
                d_pose_str = f"{'--':>10}"

            # Print Line
            print(f"{d_sys:>10.3f} | {d_ros:>10.3f} | {d_imu_str} | {d_cam_str} | {d_pose_str}")

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = SyncMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
