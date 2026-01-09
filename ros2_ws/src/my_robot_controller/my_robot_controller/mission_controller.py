#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, String, Int32
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time

# --- STATES ---
STATE_IDLE = 0        
STATE_NAV_A = 1       # Go to Cube Zone
STATE_SEARCH_A = 2    # Find Cube
STATE_APPROACH_A = 3  # Move to Cube
STATE_PICK = 4        # Grab Cube

STATE_NAV_B = 5       # Go to Platform Zone
STATE_SEARCH_B = 6    # Find Platform
STATE_APPROACH_B = 7  # Move to Platform
STATE_PLACE = 8       # Drop Cube

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- CONFIGURATION ---
        self.pos_a_x = -0.0355; self.pos_a_y = 5.8193 # Cube Zone
        self.pos_b_x = 1.3364;  self.pos_b_y = 5.5942 # Platform Zone
        
        # --- LOOP CONFIGURATION ---
        self.max_mission_loops = 3 
        self.current_loop_count = 0

        # --- SPEED & DISTANCE SETTINGS ---
        self.max_approach_speed = 0.4   
        self.min_approach_speed = 0.08  
        self.max_turn_kp = 0.002
        self.min_turn_kp = 0.001
        self.slowdown_radius = 600.0    
        
        # Stop distances
        self.stop_distance_cube = 190.0     
        self.stop_distance_platform = 110.0 
        self.stable_stop_count = 0
        self.required_stop_counts = 3

        # --- PUBS/SUBS ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vision_mode_pub = self.create_publisher(Int32, '/vision_mode', 10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_cmd', 10)
        
        self.create_subscription(Float32MultiArray, '/target_info', self.vision_callback, 10)
        self.create_subscription(Float32MultiArray, '/robot_status', self.sensor_callback, 10)
        self.create_subscription(String, '/mission_trigger', self.trigger_callback, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- VARIABLES ---
        self.current_state = STATE_IDLE
        self.target_found = False
        self.error_x = 0.0
        self.tof_dist = 9999.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Mission Ready. Will run {self.max_mission_loops} times.")

    def trigger_callback(self, msg):
        if msg.data == "start" and self.current_state == STATE_IDLE:
            self.get_logger().info("STARTING MISSION...")
            self.current_loop_count = 0 # Reset counter on manual start
            self.start_sequence()

    def start_sequence(self):
        # 1. Reset Servos
        self.send_gripper_cmd(3) # 3 = RESET
        time.sleep(1.0) 
        self.send_gripper_cmd(0)
        time.sleep(1.0)

        # 2. Go to Point A (Cube)
        # Orientation: z=0.9225, w=0.3860
        self.send_nav_goal(self.pos_a_x, self.pos_a_y, 0.9225, 0.3860)
        self.current_state = STATE_NAV_A

    def send_gripper_cmd(self, val):
        msg = Int32(); msg.data = val
        self.gripper_pub.publish(msg)

    def set_vision_mode(self, mode):
        # 0=Idle, 1=Cube, 2=Platform
        msg = Int32(); msg.data = mode
        self.vision_mode_pub.publish(msg)

    def vision_callback(self, msg):
        if len(msg.data) >= 2:
            self.target_found = bool(msg.data[0])
            self.error_x = msg.data[1]

    def sensor_callback(self, msg):
        if len(msg.data) >= 2: self.tof_dist = msg.data[1]

    # --- NAVIGATION HELPER ---
    def send_nav_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w
        
        self.get_logger().info(f"Navigating to {x}, {y}...")
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected! Retrying...")
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("DESTINATION REACHED!")
            
            # --- LOGIC SWITCHER ---
            if self.current_state == STATE_NAV_A:
                self.get_logger().info("Searching for CUBE...")
                self.set_vision_mode(1) # Enable Cube Model
                self.current_state = STATE_SEARCH_A
            
            elif self.current_state == STATE_NAV_B:
                self.get_logger().info("Searching for PLATFORM...")
                self.set_vision_mode(2) # Enable Platform Model
                self.current_state = STATE_SEARCH_B

        else:
            self.get_logger().warn(f"Navigation Failed (Status: {status})... Retrying...")
            if self.current_state == STATE_NAV_A:
                self.send_nav_goal(self.pos_a_x, self.pos_a_y, 0.9225, 0.3860)
                
            elif self.current_state == STATE_NAV_B:
                self.send_nav_goal(self.pos_b_x, self.pos_b_y, 0.6208, 0.7840)

    # --- HELPER: CALCULATE SPEED ---
    def get_approach_speed(self, distance, stop_dist):
        if distance > self.slowdown_radius:
            return self.max_approach_speed
        else:
            ratio = (distance - stop_dist) / (self.slowdown_radius - stop_dist)
            ratio = max(0.0, min(1.0, ratio)) 
            return self.min_approach_speed + (self.max_approach_speed - self.min_approach_speed) * ratio
        
    def get_turn_kp(self, distance, stop_dist):
        if distance > self.slowdown_radius:
            return self.max_turn_kp 
        else:
            ratio = (distance - stop_dist) / (self.slowdown_radius - stop_dist)
            ratio = max(0.0, min(1.0, ratio))
            return self.min_turn_kp + (self.max_turn_kp - self.min_turn_kp) * ratio

    # --- MAIN LOOP ---
    def control_loop(self):
        cmd = Twist()

        # === PHASE 1: PICK CUBE ===
        if self.current_state == STATE_SEARCH_A:
            self.stable_stop_count = 0
            if self.target_found:
                self.current_state = STATE_APPROACH_A
            else:
                cmd.angular.z = 0.2 # Spin to find cube

        elif self.current_state == STATE_APPROACH_A:
            if not self.target_found:
                self.current_state = STATE_SEARCH_A
                return
            
            current_turn_kp = self.get_turn_kp(self.tof_dist, self.stop_distance_cube)
            cmd.angular.z = self.error_x * current_turn_kp

            if abs(self.error_x) < 20: 
                if self.tof_dist <= self.stop_distance_cube:
                    self.stable_stop_count += 1
                else:
                    self.stable_stop_count = 0
                    cmd.linear.x = self.get_approach_speed(self.tof_dist, self.stop_distance_cube)

                if self.stable_stop_count >= self.required_stop_counts:
                    self.get_logger().info("STABLE TARGET REACHED (Pick)")
                    self.cmd_pub.publish(Twist()) # Force Stop
                    self.current_state = STATE_PICK
                    self.stable_stop_count = 0 # Reset for next time

        elif self.current_state == STATE_PICK:
            self.get_logger().info("ACTION: PICKING CUBE")
            self.set_vision_mode(0) # Stop Vision
            self.send_gripper_cmd(1) # 1 = PICK
            time.sleep(0.1)
            self.send_gripper_cmd(0)
            
            time.sleep(3.0) 
            
            self.get_logger().info("Moving to Point B...")
            self.send_nav_goal(self.pos_b_x, self.pos_b_y, 0.6208, 0.7840)
            self.current_state = STATE_NAV_B

        # === PHASE 2: PLACE ON PLATFORM ===
        elif self.current_state == STATE_SEARCH_B:
            if self.target_found:
                self.current_state = STATE_APPROACH_B
            else:
                cmd.angular.z = -0.2 # Spin to find platform

        elif self.current_state == STATE_APPROACH_B:
            if not self.target_found:
                self.current_state = STATE_SEARCH_B
                return
            
            current_turn_kp = self.get_turn_kp(self.tof_dist, self.stop_distance_platform)
            cmd.angular.z = self.error_x * current_turn_kp

            if abs(self.error_x) < 20:
                if self.tof_dist <= self.stop_distance_platform:
                    self.stable_stop_count += 1
                else:
                    self.stable_stop_count = 0 # Reset
                    cmd.linear.x = self.get_approach_speed(self.tof_dist, self.stop_distance_platform)

                if self.stable_stop_count >= self.required_stop_counts:
                    self.get_logger().info("STABLE TARGET REACHED (Place)")
                    self.cmd_pub.publish(Twist()) # Force Stop
                    self.current_state = STATE_PLACE
                    self.stable_stop_count = 0

        elif self.current_state == STATE_PLACE:
            self.get_logger().info("ACTION: PLACING CUBE")
            self.set_vision_mode(0) # Stop Vision
            self.send_gripper_cmd(2) # 2 = PLACE
            time.sleep(0.1)
            self.send_gripper_cmd(0)

            time.sleep(3.0)
            
            # --- [NEW] LOOP CHECK LOGIC ---
            self.current_loop_count += 1
            self.get_logger().info(f"Mission Loop {self.current_loop_count}/{self.max_mission_loops} Complete.")

            if self.current_loop_count < self.max_mission_loops:
                self.get_logger().info("Restarting Sequence...")
                self.start_sequence() # Go back to Point A
            else:
                self.get_logger().info("ALL MISSIONS COMPLETE. STOPPING.")
                self.current_state = STATE_IDLE

        # Publish Velocity
        if self.current_state in [STATE_SEARCH_A, STATE_APPROACH_A, STATE_SEARCH_B, STATE_APPROACH_B]:
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()