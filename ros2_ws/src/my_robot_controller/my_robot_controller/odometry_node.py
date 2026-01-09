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
        
        # --- STATE VARIABLES ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Current velocities (updated by callback)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # --- IMU SETUP ---
        self.initial_heading = None  

        # --- ROS COMMUNICATION ---
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/robot_status',
            self.status_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- TIMER SETUP (The Fix) ---
        # Run at 20Hz (0.05s) to keep TF alive and smooth
        self.timer = self.create_timer(0.05, self.publish_loop)
        self.last_time = self.get_clock().now()

        self.get_logger().info("Robust Odometry Node Started")

    def status_callback(self, msg):
        # Message Format: [0]=Heading, [1]=ToF, [2]=M1, [3]=M2, [4]=M3, [5]=M4
        if len(msg.data) < 6: return

        # 1. READ IMU (The "Truth" for Heading)
        raw_heading_degrees = msg.data[0]
        current_imu_rad = raw_heading_degrees * (math.pi / 180.0)

        # Handle "Zeroing" on startup
        if self.initial_heading is None:
            self.initial_heading = current_imu_rad
        
        # Update Theta (Inverted for ROS standard)
        self.th = -(current_imu_rad - self.initial_heading)

        # Wrap Angle (-PI to PI)
        while self.th > math.pi: self.th -= 2 * math.pi
        while self.th < -math.pi: self.th += 2 * math.pi

        # 2. READ WHEELS (For Speed)
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
        vx = (v_fr + v_rr - v_rl - v_fl) / 4.0
        vy = (v_rr + v_rl - v_fr - v_fl) / 4.0
        
        # Calculate WZ for reporting
        geometry_sum = 4.0 * (self.lx + self.ly)
        wz_wheels = (v_fr + v_rr + v_rl + v_fl) / geometry_sum

        # Apply Inversion & Correction
        self.current_vx = -vx * self.x_correction
        self.current_vy = vy * self.y_correction
        self.current_wz = -wz_wheels

    def publish_loop(self):
        # This function runs constantly at 20Hz
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. INTEGRATE POSITION (Dead Reckoning)
        # Use the latest known velocity and heading
        delta_x = (self.current_vx * cos(self.th) - self.current_vy * sin(self.th)) * dt
        delta_y = (self.current_vx * sin(self.th) + self.current_vy * cos(self.th)) * dt
        
        self.x += delta_x
        self.y += delta_y

        if isnan(self.x) or isnan(self.y):
            self.x = 0.0; self.y = 0.0

        # 2. PREPARE TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 3. BROADCAST TF
        self.tf_broadcaster.sendTransform(t)

        # 4. PUBLISH ODOMETRY
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation

        odom.twist.twist.linear.x = self.current_vx
        odom.twist.twist.linear.y = self.current_vy
        odom.twist.twist.angular.z = self.current_wz

        self.odom_pub.publish(odom)

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