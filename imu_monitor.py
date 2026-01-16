#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Imu

# Standard Earth gravity (Reaction force magnitude)
GRAVITY_STD = -9.80665

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def get_expected_gravity_vector(x, y, z, w):
    """
    Calculates the expected gravity vector [gx, gy, gz] in the SENSOR frame.
    
    UPDATED LOGIC for ORB-SLAM3 / OpenCV:
    1. We assume 'World Reaction Force' points along World -Y: [0, -1, 0].
       (Because in OpenCV frames, Y is Down, so Reaction Force is Up/Negative Y).
    2. We rotate this vector [0, -1, 0] into the SENSOR frame using the 
       inverse of the orientation quaternion.
    """
    # 1. Normalize quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0: return 0,0,0
    x /= norm; y /= norm; z /= norm; w /= norm

    # 2. Project World Vector (0, -1, 0) into Body Frame.
    # We calculate the 2nd row of the Rotation Matrix (R_body->world)
    # and negate it (because we want R_world->body * [0, -1, 0]).
    
    # R_row1 (Corresponds to Y mapping)
    # r10 = 2(xy + zw)
    # r11 = 1 - 2(xx + zz)
    # r12 = 2(yz - xw)
    
    # Since Target World Vector is [0, -1, 0], we negate these terms:
    gx = -2.0 * (x * y + z * w)
    gy = -(1.0 - 2.0 * (x * x + z * z))
    gz = -2.0 * (y * z - x * w)

    # Scale by standard gravity magnitude
    return (gx * GRAVITY_STD, gy * GRAVITY_STD, gz * GRAVITY_STD)

class ImuGravityMonitor:
    def __init__(self):
        rospy.init_node('imu_gravity_monitor', anonymous=True)
        self.sub = rospy.Subscriber('/imu/data', Imu, self.callback)
        self.last_print = rospy.Time.now()
        self.print_interval = rospy.Duration(0.2) 

    def callback(self, data):
        if (rospy.Time.now() - self.last_print) < self.print_interval:
            return
        self.last_print = rospy.Time.now()

        # --- 1. ORIENTATION ---
        qx, qy, qz, qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        
        # --- 2. MEASURED ACCEL ---
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z
        
        # --- 3. EXPECTED GRAVITY ---
        ex, ey, ez = get_expected_gravity_vector(qx, qy, qz, qw)
        
        # --- 4. PRINTING ---
        print("\033c") 
        print("======== IMU GRAVITY ANALYZER (Ref: -Y) ========")
        print(f"Topic: /imu/data")
        print("Assumption: World 'Up' is vector [0, -1, 0]")
        print("---------------------------------------------")
        print(f"ORIENTATION:")
        print(f"  Roll:  {math.degrees(roll):8.2f}°")
        print(f"  Pitch: {math.degrees(pitch):8.2f}°")
        print(f"  Yaw:   {math.degrees(yaw):8.2f}°")
        print("---------------------------------------------")
        print(f"GRAVITY VECTOR COMPARISON (m/s^2):")
        print(f"     | {'MEASURED':^10} | {'EXPECTED':^10} | {'ERROR':^10}")
        print(f"  X  | {ax:10.4f} | {ex:10.4f} | {abs(ax-ex):10.4f}")
        print(f"  Y  | {ay:10.4f} | {ey:10.4f} | {abs(ay-ey):10.4f}")
        print(f"  Z  | {az:10.4f} | {ez:10.4f} | {abs(az-ez):10.4f}")
        print("---------------------------------------------")
        print("INTERPRETATION:")
        
        total_error = math.sqrt((ax-ex)**2 + (ay-ey)**2 + (az-ez)**2)
        if total_error < 1.0: # Increased threshold slightly for different conventions
            print(f"  Status: STATIONARY / ALIGNED")
        else:
            print(f"  Status: MOVING / UNCALIBRATED")
            print(f"  Residual: {total_error:.4f} m/s^2")
            
        print("=============================================")
        print("Press Ctrl+C to exit")

if __name__ == '__main__':
    try:
        monitor = ImuGravityMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
