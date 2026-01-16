#!/usr/bin/env python3
import rospy
import argparse
import yaml
import os
import math
import numpy as np
from sensor_msgs.msg import Imu
from threading import Lock

# Standard Earth gravity (Reaction force magnitude)
GRAVITY_STD = -9.83

def get_expected_gravity_vector(x, y, z, w):
    """
    Calculates the expected gravity vector [gx, gy, gz] in the SENSOR frame.
    Assumes World Reaction Force is along -Y (OpenCV/ORB-SLAM convention).
    """
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0: return np.array([0., 0., 0.])
    x /= norm; y /= norm; z /= norm; w /= norm

    # Project World Vector (0, -1, 0) into Body Frame via inverse rotation
    gx = -2.0 * (x * y + z * w)
    gy = -(1.0 - 2.0 * (x * x + z * z))
    gz = -2.0 * (y * z - x * w)

    return np.array([gx * GRAVITY_STD, gy * GRAVITY_STD, gz * GRAVITY_STD])

class RunningStats:
    """Helper to calculate online Mean and Std Dev."""
    def __init__(self, dim=3):
        self.n = 0
        self.dim = dim
        self.mean = np.zeros(dim)
        self.M2 = np.zeros(dim)

    def update(self, x):
        x = np.array(x)
        if x.shape != self.mean.shape:
            # Handle scalar vs vector wrapping
            if self.dim == 1: x = np.array([x])
            
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2

    def get_mean(self):
        return self.mean

    def get_std(self):
        if self.n < 2:
            return np.zeros_like(self.mean)
        return np.sqrt(self.M2 / (self.n - 1))

class ImuRecorder:
    def __init__(self, filepath):
        self.filepath = filepath
        self.lock = Lock()
        
        # Stats accumulators
        self.acc_raw_stats = RunningStats(3)   # Measured raw vector
        self.acc_norm_stats = RunningStats(1)  # Magnitude of acceleration
        self.acc_bias_stats = RunningStats(3)  # Residual (Measured - Expected)
        self.gyro_bias_stats = RunningStats(3)
        self.orient_stats = RunningStats(4) 
        
        # ROS
        rospy.init_node('imu_bias_recorder', anonymous=True)
        self.sub = rospy.Subscriber('/imu/data', Imu, self.callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.save_callback)
        
        rospy.loginfo(f"Recording IMU stats to {self.filepath}...")
        rospy.loginfo(f"Using Standard Gravity Ref: {GRAVITY_STD}")
        rospy.loginfo("Keep the robot STATIONARY. Ctrl+C to stop.")

    def callback(self, msg):
        with self.lock:
            # 1. Orientation
            qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            self.orient_stats.update([qx, qy, qz, qw])

            # 2. Linear Accel
            measured_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            acc_norm = np.linalg.norm(measured_acc)
            
            # Expected gravity based on current orientation
            expected_gravity = get_expected_gravity_vector(qx, qy, qz, qw)
            
            # Bias = Measured - Expected
            acc_bias = measured_acc - expected_gravity
            
            # Update Stats
            self.acc_raw_stats.update(measured_acc)
            self.acc_norm_stats.update(acc_norm)
            self.acc_bias_stats.update(acc_bias)
            
            # 3. Gyro
            gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            self.gyro_bias_stats.update(gyro)

    def save_callback(self, event):
        with self.lock:
            if self.acc_bias_stats.n == 0:
                return
            
            # Get current stats
            raw_mean = self.acc_raw_stats.get_mean()
            norm_mean = self.acc_norm_stats.get_mean()[0] # scalar
            norm_std = self.acc_norm_stats.get_std()[0]   # scalar
            bias_mean = self.acc_bias_stats.get_mean()
            bias_std = self.acc_bias_stats.get_std()
            gyro_mean = self.gyro_bias_stats.get_mean()
            gyro_std = self.gyro_bias_stats.get_std()
            orient_mean = self.orient_stats.get_mean()
            
            # Calculate average expected gravity for display (Consistency check)
            # We re-calculate it from the mean orientation to see the "Average Target"
            avg_expected_g = get_expected_gravity_vector(*orient_mean)

            data = {
                'count': self.acc_bias_stats.n,
                'accel_norm': {
                    'mean': float(norm_mean),
                    'std': float(norm_std)
                },
                'accel_bias': {
                    'mean': bias_mean.tolist(),
                    'std': bias_std.tolist()
                },
                'gyro_bias': {
                    'mean': gyro_mean.tolist(),
                    'std': gyro_std.tolist()
                },
                'orientation': {
                    'mean': orient_mean.tolist(),
                    'std': self.orient_stats.get_std().tolist()
                }
            }

        # Clear screen and print status
        # print("\033c", end='') # Optional: Un-comment to clear terminal history on every update
        print("-" * 60)
        print(f"Samples Recorded: {data['count']}")
        print(f"ORIENTATION (xyzw): {np.round(orient_mean, 3)}")
        print("-" * 60)
        
        print(f"ACCELEROMETER MAGNITUDE (Estimation of Gravity):")
        print(f"  Measured Norm:   {norm_mean:.5f} m/s^2  (Std: {norm_std:.5f})")
        print(f"  Reference Norm:  {abs(GRAVITY_STD):.5f} m/s^2")
        
        print(f"\nACCELEROMETER VECTORS (m/s^2):")
        print(f"  Measured (Avg):  [{raw_mean[0]:.4f}, {raw_mean[1]:.4f}, {raw_mean[2]:.4f}]")
        print(f"  Expected (Avg):  [{avg_expected_g[0]:.4f}, {avg_expected_g[1]:.4f}, {avg_expected_g[2]:.4f}]")
        print(f"  Resulting Bias:  [{bias_mean[0]:.4f}, {bias_mean[1]:.4f}, {bias_mean[2]:.4f}]")
        print(f"  Bias Std Dev:    [{bias_std[0]:.4f}, {bias_std[1]:.4f}, {bias_std[2]:.4f}]")

        print(f"\nGYROSCOPE BIAS (rad/s):")
        print(f"  Mean:            [{gyro_mean[0]:.5f}, {gyro_mean[1]:.5f}, {gyro_mean[2]:.5f}]")
        print(f"  Std:             [{gyro_std[0]:.5f}, {gyro_std[1]:.5f}, {gyro_std[2]:.5f}]")

        # Dump to YAML
        with open(self.filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=None)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record IMU bias stats.')
    parser.add_argument('filename', type=str, help='Path to output yaml file')
    args = parser.parse_args()

    # Ensure directory exists
    directory = os.path.dirname(args.filename)
    if directory:
        os.makedirs(directory, exist_ok=True)

    try:
        recorder = ImuRecorder(args.filename)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
