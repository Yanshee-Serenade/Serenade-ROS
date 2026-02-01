#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf.transformations as tf_trans
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ubt_msgs.msg import gyro_report

class ImuCameraAligner:
    def __init__(self):
        rospy.init_node('imu_camera_aligner_node', anonymous=True)

        # --- Constants ---
        self.G_TO_M_PER_S2 = 9.80665
        self.DEG_TO_RAD = math.pi / 180.0
        # Updated frame_id to reflect that this is now Camera-aligned data
        self.TARGET_FRAME_ID = "camera" 
        self.GYROSCOPE_BIAS = [-0.01994, -0.00107, 0.00486]

        # --- Noise Parameters ---
        acc_noise_density = 0.016440119091087155
        gyro_noise_density = 0.0013994966890025684
        update_rate = 19.0

        self.acc_variance = (acc_noise_density ** 2) * update_rate
        self.gyro_variance = (gyro_noise_density ** 2) * update_rate

        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.sub = rospy.Subscriber('/hal_gyro_report', gyro_report, self.callback)

        # --- Rotation Definition ---
        # Matrix representing rotation from RDF (Right-Down-Forward) to Camera Frame
        # based on: X->-X, Y->Z, Z->Y
        # Row 0: New X axis in Old terms (-1, 0, 0)
        # Row 1: New Y axis in Old terms (0, 0, 1)
        # Row 2: New Z axis in Old terms (0, 1, 0)
        rotation_matrix = np.array([
            [-1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        self.q_correction = tf_trans.quaternion_from_matrix(rotation_matrix)

        rospy.loginfo("IMU Aligner Started: Outputting data in Camera Frame")

    def remap_to_rdf(self, x, y, z):
        """
        Original Step 1: Maps LBU vector components to RDF vector components.
        """
        new_x = -x
        new_y = -z
        new_z = -y
        return new_x, new_y, new_z

    def remap_rdf_to_camera(self, x, y, z):
        """
        Step 2: Maps RDF (Right-Down-Forward) to Camera Frame.
        Rules:
        Gyro Right (X) -> Cam Left (-X)
        Gyro Down  (Y) -> Cam Fwd  (Z)
        Gyro Fwd   (Z) -> Cam Down (Y) [Derived from Back->Up]
        """
        cam_x = -x
        cam_y = z
        cam_z = y
        return cam_x, cam_y, cam_z

    def callback(self, msg):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.TARGET_FRAME_ID

            # --- 1. Linear Acceleration ---
            # Raw data is in G's, convert to m/s^2
            ax_raw = -msg.accel_data[0] * self.G_TO_M_PER_S2
            ay_raw = -msg.accel_data[1] * self.G_TO_M_PER_S2
            az_raw = -msg.accel_data[2] * self.G_TO_M_PER_S2
            
            # Step A: LBU -> RDF
            ax_rdf, ay_rdf, az_rdf = self.remap_to_rdf(ax_raw, ay_raw, az_raw)
            # Step B: RDF -> Camera
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = \
                self.remap_rdf_to_camera(ax_rdf, ay_rdf, az_rdf)

            # --- 2. Angular Velocity ---
            # Raw data is deg/s, convert to rad/s
            gx_raw = msg.gyro_data[0] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[0]
            gy_raw = msg.gyro_data[1] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[1]
            gz_raw = msg.gyro_data[2] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[2]

            # Step A: LBU -> RDF
            gx_rdf, gy_rdf, gz_rdf = self.remap_to_rdf(gx_raw, gy_raw, gz_raw)
            # Step B: RDF -> Camera
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = \
                self.remap_rdf_to_camera(gx_rdf, gy_rdf, gz_rdf)

            # --- 3. Orientation ---
            if hasattr(msg, 'euler_data') and len(msg.euler_data) >= 3:
                # 1. Get Quaternion in ORIGINAL LBU frame
                q_raw = tf_trans.quaternion_from_euler(
                    msg.euler_data[0] * self.DEG_TO_RAD,
                    msg.euler_data[1] * self.DEG_TO_RAD,
                    msg.euler_data[2] * self.DEG_TO_RAD
                )
                
                # 2. Convert to RDF (Using the legacy manual mapping from original script)
                # Rule from original script: x -> -x, y -> -z, z -> -y
                q_rdf = np.array([
                    -q_raw[0], # x
                    -q_raw[2], # y (was z)
                    -q_raw[1], # z (was y)
                    q_raw[3]   # w
                ])

                # 3. Apply Rotation to Camera Frame
                # Rotate the RDF quaternion by the RDF->Camera correction quaternion
                q_cam = tf_trans.quaternion_multiply(q_rdf, self.q_correction)

                imu_msg.orientation = Quaternion(*q_cam)
                
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            else:
                imu_msg.orientation_covariance[0] = -1

            # --- 4. Covariances ---
            # Since variances are scalar and identical on axes (spherical noise), 
            # rotation doesn't change the values on the diagonal.
            imu_msg.linear_acceleration_covariance = [self.acc_variance, 0, 0, 0, self.acc_variance, 0, 0, 0, self.acc_variance]
            imu_msg.angular_velocity_covariance = [self.gyro_variance, 0, 0, 0, self.gyro_variance, 0, 0, 0, self.gyro_variance]

            self.pub.publish(imu_msg)

        except Exception as e:
            rospy.logerr_throttle(1, f"Conversion Error: {e}")

if __name__ == '__main__':
    try:
        node = ImuCameraAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass