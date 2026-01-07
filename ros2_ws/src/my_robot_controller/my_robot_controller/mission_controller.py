#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose

# --- STATES ---
STATE_IDLE = 0        # Waiting for user command
STATE_NAVIGATING = 1  # Driving to the Search Zone
STATE_SEARCH = 2      # Spinning to find object
STATE_APPROACH = 3    # Visual Servoing
STATE_GRAB = 4        # Simulated Grabbing

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- CONFIGURATION ---
        # "Search Zone" Coordinates (Change these to your Map Point!)
        self.search_zone_x = 3.31
        self.search_zone_y = 1.48

        # --- PUBS/SUBS ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracker_trigger_pub = self.create_publisher(Bool, '/enable_tracking', 10)
        
        self.create_subscription(Float32MultiArray, '/target_info', self.vision_callback, 10)
        self.create_subscription(Float32MultiArray, '/robot_status', self.sensor_callback, 10)
        
        # New: Trigger Topic (Send "start" here to begin)
        self.create_subscription(String, '/mission_trigger', self.trigger_callback, 10)

        # --- NAV2 ACTION CLIENT ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- VARIABLES ---
        self.current_state = STATE_IDLE
        self.target_found = False
        self.error_x = 0.0
        self.tof_dist = 9999.0
        self.turn_kp = 0.005
        self.stop_distance = 170.0

        # Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Mission Controller Ready. Waiting for '/mission_trigger'...")

    def trigger_callback(self, msg):
        if msg.data == "start" and self.current_state == STATE_IDLE:
            self.get_logger().info("RECEIVED START COMMAND!")
            self.start_navigation()

    def vision_callback(self, msg):
        if len(msg.data) >= 3:
            self.target_found = bool(msg.data[0])
            self.error_x = msg.data[1]

    def sensor_callback(self, msg):
        if len(msg.data) >= 2:
            self.tof_dist = msg.data[1]

    # --- NAVIGATION LOGIC ---
    def start_navigation(self):
        self.current_state = STATE_NAVIGATING
        self.get_logger().info(f"Navigating to Search Zone ({self.search_zone_x}, {self.search_zone_y})...")

        # 1. Wait for Nav2
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available! Is Navigation running?")
            self.current_state = STATE_IDLE
            return

        # 2. Create Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.search_zone_x
        goal_msg.pose.pose.position.y = self.search_zone_y
        goal_msg.pose.pose.orientation.w = 1.0 # Face East

        # 3. Send Goal
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            self.current_state = STATE_IDLE
            return

        self.get_logger().info("Goal accepted! Driving...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # This triggers when the robot Arrives at the zone
        self.get_logger().info("ARRIVED AT SEARCH ZONE!")
        
        # 1. Enable the AI Tracker
        msg = Bool()
        msg.data = True
        self.tracker_trigger_pub.publish(msg)
        
        # 2. Switch State
        self.current_state = STATE_SEARCH

    # --- CONTROL LOOP ---
    def control_loop(self):
        cmd = Twist()

        if self.current_state == STATE_IDLE:
            pass # Do nothing, wait for trigger

        elif self.current_state == STATE_NAVIGATING:
            pass # Do nothing, Nav2 is driving the wheels

        elif self.current_state == STATE_SEARCH:
            if self.target_found:
                self.get_logger().info("TARGET SPOTTED! Approaching...")
                cmd.angular.z = 0.0
                self.current_state = STATE_APPROACH
            else:
                cmd.angular.z = 0.3 # Spin to find it

        elif self.current_state == STATE_APPROACH:
            if not self.target_found:
                self.current_state = STATE_SEARCH
                return

            cmd.angular.z = self.error_x * self.turn_kp

            if abs(self.error_x) < 50:
                if self.tof_dist > self.stop_distance:
                    cmd.linear.x = 0.2
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.current_state = STATE_GRAB
            else:
                cmd.linear.x = 0.0

        elif self.current_state == STATE_GRAB:
            self.get_logger().info("GRABBING... (Mission Complete for now)")
            # Stop AI to save CPU
            msg = Bool()
            msg.data = False
            self.tracker_trigger_pub.publish(msg)
            
            # Logic to reset or continue...
            self.current_state = STATE_IDLE 

        if self.current_state in [STATE_SEARCH, STATE_APPROACH, STATE_GRAB]:
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()