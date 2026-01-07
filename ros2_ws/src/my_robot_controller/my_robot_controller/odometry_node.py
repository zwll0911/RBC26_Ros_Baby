#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from math import sin, cos, pi, isnan

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- ROBOT PHYSICAL CONSTANTS ---
        self.wheel_radius = 0.046   
        self.lx = 0.23              
        self.ly = 0.20    

        # --- CALIBRATION MULTIPLIERS ---   
        self.x_correction = 0.60
        self.y_correction = 0.40       
        
        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # --- IMU SETUP ---
        self.initial_heading = None  # To store the starting angle

        # --- ROS COMMUNICATION ---
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/robot_status',
            self.status_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def status_callback(self, msg):
        # Message Format: [0]=Heading, [1]=ToF, [2]=M1, [3]=M2, [4]=M3, [5]=M4
        if len(msg.data) < 6: return

        # ---------------------------------------------------------
        # 1. READ IMU (The "Truth" for Heading)
        # ---------------------------------------------------------
        # Your ESP32 sends Degrees (-180 to 180)
        raw_heading_degrees = msg.data[0]
        
        # Convert to Radians
        current_imu_rad = raw_heading_degrees * (math.pi / 180.0)

        # Handle "Zeroing" on startup
        if self.initial_heading is None:
            self.initial_heading = current_imu_rad
        
        # Calculate Theta relative to start
        # ROS Standard: Counter-Clockwise is Positive (+).
        # BNO Standard: Clockwise is Positive (+).
        # FIX: We add a negative sign to flip the direction.
        self.th = -(current_imu_rad - self.initial_heading)

        # Ensure we stay in -PI to +PI range (just in case)
        # (Though your ESP32 code handles this well, this is a safety net)
        while self.th > math.pi: self.th -= 2 * math.pi
        while self.th < -math.pi: self.th += 2 * math.pi

        # ---------------------------------------------------------
        # 2. READ WHEELS (For X/Y Speed Only)
        # ---------------------------------------------------------
        rpm_m1_fr = msg.data[2]
        rpm_m2_rr = msg.data[3]
        rpm_m3_rl = msg.data[4]
        rpm_m4_fl = msg.data[5]

        if isnan(rpm_m1_fr) or isnan(rpm_m2_rr) or isnan(rpm_m3_rl) or isnan(rpm_m4_fl):
             return

        factor = 0.10472 * self.wheel_radius
        v_fr = rpm_m1_fr * factor
        v_rr = rpm_m2_rr * factor
        v_rl = rpm_m3_rl * factor
        v_fl = rpm_m4_fl * factor

        # 3. CALCULATE LOCAL VELOCITY
        # We only care about VX and VY from wheels.
        vx = (v_fr + v_rr - v_rl - v_fl) / 4.0
        vy = (v_rr + v_rl - v_fr - v_fl) / 4.0
        
        # Calculate WZ from wheels just for reporting (optional)
        geometry_sum = 4.0 * (self.lx + self.ly)
        wz_wheels = (v_fr + v_rr + v_rl + v_fl) / geometry_sum

        # IMPORTANT: Apply logic inversion if your motors are physically reversed
        vx = -vx 
        wz_wheels = -wz_wheels 

        # Slip Correction
        vx = vx * self.x_correction
        vy = vy * self.y_correction

        # 4. INTEGRATE POSITION (X/Y)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Use the IMU heading (self.th) to rotate the velocity vector
        # This is the "Magic" that makes Mecanum drift-proof
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        
        self.x += delta_x
        self.y += delta_y

        if isnan(self.x) or isnan(self.y):
            self.x = 0.0
            self.y = 0.0

        current_time_msg = current_time.to_msg()

        # ---------------------------------------------------------
        # 5. PUBLISH
        # ---------------------------------------------------------
        odom = Odometry()
        odom.header.stamp = current_time_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (From IMU)
        q = self.euler_to_quaternion(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity 
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz_wheels

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = current_time_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()