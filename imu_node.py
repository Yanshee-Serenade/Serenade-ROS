#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf.transformations as tf_trans
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ubt_msgs.msg import gyro_report

class ImuAxisRemapper:
    def __init__(self):
        rospy.init_node('imu_remapper_node', anonymous=True)

        # --- Constants ---
        self.G_TO_M_PER_S2 = 9.80665
        self.DEG_TO_RAD = math.pi / 180.0
        self.TARGET_FRAME_ID = "imu_link"
        self.GYROSCOPE_BIAS = [-0.01994, -0.00107, 0.00486]

        # --- Noise Parameters ---
        acc_noise_density = 0.016440119091087155
        gyro_noise_density = 0.0013994966890025684
        update_rate = 19.0

        self.acc_variance = (acc_noise_density ** 2) * update_rate
        self.gyro_variance = (gyro_noise_density ** 2) * update_rate

        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.sub = rospy.Subscriber('/hal_gyro_report', gyro_report, self.callback)

        rospy.loginfo("IMU Remapper Started: Swapping axes from LBU to RDF (Identity preserved)")

    def remap_vector(self, x, y, z):
        """
        Maps LBU vector components to RDF vector components.
        Rule: 
        RDF_Right (X)   = -LBU_Left (X)
        RDF_Down (Y)    = -LBU_Up (Z)
        RDF_Forward (Z) = -LBU_Back (Y)
        """
        new_x = -x
        new_y = -z
        new_z = -y
        return new_x, new_y, new_z

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
            
            # Apply Remapping
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = \
                self.remap_vector(ax_raw, ay_raw, az_raw)

            # --- 2. Angular Velocity ---
            # Raw data is deg/s, convert to rad/s
            gx_raw = msg.gyro_data[0] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[0]
            gy_raw = msg.gyro_data[1] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[1]
            gz_raw = msg.gyro_data[2] * self.DEG_TO_RAD - self.GYROSCOPE_BIAS[2]

            # Apply Remapping
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = \
                self.remap_vector(gx_raw, gy_raw, gz_raw)

            # --- 3. Orientation ---
            if hasattr(msg, 'euler_data') and len(msg.euler_data) >= 3:
                # First, get the quaternion in the ORIGINAL LBU frame
                # ROS quaternion_from_euler takes (roll, pitch, yaw)
                q_raw = tf_trans.quaternion_from_euler(
                    msg.euler_data[0] * self.DEG_TO_RAD,
                    msg.euler_data[1] * self.DEG_TO_RAD,
                    msg.euler_data[2] * self.DEG_TO_RAD
                )
                
                # q_raw is (x, y, z, w)
                # We simply apply the same vector remapping to the x, y, z components of the quaternion.
                # w (scalar) remains untouched.
                
                # Rule: x -> -x, y -> -z, z -> -y
                new_q_x = -q_raw[0]
                new_q_y = -q_raw[2] # Old Z becomes New Y
                new_q_z = -q_raw[1] # Old Y becomes New Z
                new_q_w = q_raw[3]

                imu_msg.orientation = Quaternion(new_q_x, new_q_y, new_q_z, new_q_w)
                
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            else:
                imu_msg.orientation_covariance[0] = -1

            # --- 4. Covariances ---
            imu_msg.linear_acceleration_covariance = [self.acc_variance, 0, 0, 0, self.acc_variance, 0, 0, 0, self.acc_variance]
            imu_msg.angular_velocity_covariance = [self.gyro_variance, 0, 0, 0, self.gyro_variance, 0, 0, 0, self.gyro_variance]

            self.pub.publish(imu_msg)

        except Exception as e:
            rospy.logerr_throttle(1, f"Conversion Error: {e}")

if __name__ == '__main__':
    try:
        node = ImuAxisRemapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
