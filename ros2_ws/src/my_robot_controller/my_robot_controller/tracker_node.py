#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge      
import cv2
import numpy as np
import onnxruntime as ort
import threading
import os
import time
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURATION ---
INPUT_WIDTH = 320
INPUT_HEIGHT = 320
SCORE_THRESHOLD = 0.5 
NMS_THRESHOLD = 0.5     
SMOOTHING_FACTOR = 0.8 

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        package_share = get_package_share_directory('my_robot_controller')
        
        # --- PATHS TO MODELS ---
        self.path_cube = os.path.join(package_share, 'config', 'mrc_yellow_cude_320.onnx')
        self.path_platform = os.path.join(package_share, 'config', 'platform.onnx')
    
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_info', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug', 10)

        # --- MODE CONTROL ---
        self.vision_mode = 0 # 0=Idle, 1=Cube, 2=Platform
        self.create_subscription(Int32, '/vision_mode', self.mode_callback, 10)
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None        
        self.current_box = None         
        self.smooth_box = None      
        self.running = True             

        # --- LOAD MODELS ---
        self.session_cube = self.load_onnx_model(self.path_cube, "Cube")
        self.session_platform = self.load_onnx_model(self.path_platform, "Platform")

        self.cam_thread = threading.Thread(target=self.camera_loop)
        self.cam_thread.start()
        
        self.timer = self.create_timer(0.01, self.ai_callback)

    def load_onnx_model(self, path, name):
        """ Helper function to load ONNX models safely """
        self.get_logger().info(f"Loading {name} Model: {path}")
        try:
            sess_options = ort.SessionOptions()
            sess_options.intra_op_num_threads = 4 
            sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            session = ort.InferenceSession(path, sess_options, providers=['CPUExecutionProvider'])
            self.get_logger().info(f"{name} Model Loaded Successfully!")
            return session
        except Exception as e:
            self.get_logger().error(f"FAILED to load {name} model: {e}")
            return None

    def mode_callback(self, msg):
        self.vision_mode = msg.data
        mode_map = {0: "IDLE", 1: "CUBE", 2: "PLATFORM"}
        
        mode_name = mode_map.get(self.vision_mode, "UNKNOWN")
        self.get_logger().info(f"VISION MODE SWITCHED: {mode_name}")
        
        if self.vision_mode == 0:
            self.current_box = None

    def camera_loop(self):
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened(): return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret: continue

            # If Idle, slow down camera loop to save CPU
            if self.vision_mode == 0:
                time.sleep(0.1)
                
                # Still publish debug image saying "SLEEPING"
                cv2.putText(frame, "AI IDLE (Mode 0)", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.debug_pub.publish(ros_image)
                continue

            with self.lock:
                self.latest_frame = frame.copy()

            # --- DRAWING LOGIC (Visual Feedback) ---
            if self.current_box is not None:
                tx, ty, tw, th = self.current_box
                
                if self.smooth_box is None:
                    self.smooth_box = [tx, ty, tw, th]
                else:
                    sx, sy, sw, sh = self.smooth_box
                    nsx = sx + (tx - sx) * SMOOTHING_FACTOR
                    nsy = sy + (ty - sy) * SMOOTHING_FACTOR
                    nsw = sw + (tw - sw) * SMOOTHING_FACTOR
                    nsh = sh + (th - sh) * SMOOTHING_FACTOR
                    self.smooth_box = [nsx, nsy, nsw, nsh]

                dx, dy, dw, dh = map(int, self.smooth_box)
                
                # Color based on mode
                color = (0, 255, 255) if self.vision_mode == 1 else (255, 0, 0) # Yellow for Cube, Blue for Platform
                label = "CUBE" if self.vision_mode == 1 else "PLATFORM"

                cv2.rectangle(frame, (dx, dy), (dx + dw, dy + dh), color, 3)
                cv2.putText(frame, f"{label} LOCKED", (dx, dy - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish Debug Image
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_pub.publish(ros_image)
        
        cap.release()

    def ai_callback(self):
        if self.vision_mode == 0: return
        
        active_session = None
        if self.vision_mode == 1:
            active_session = self.session_cube
        elif self.vision_mode == 2:
            active_session = self.session_platform

        if active_session is None: return

        with self.lock:
            if self.latest_frame is None: return
            frame = self.latest_frame.copy()
        
        image_height, image_width, _ = frame.shape
        center_screen_x = image_width / 2

        # Preprocessing
        img = cv2.resize(frame, (INPUT_WIDTH, INPUT_HEIGHT))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = np.expand_dims(img, axis=0)
        img = img.astype(np.float32) / 255.0

        try:
            # Run Inference on the ACTIVE session
            input_name = active_session.get_inputs()[0].name
            output_names = [o.name for o in active_session.get_outputs()]
            outputs = active_session.run(output_names, {input_name: img})
        except Exception: return

        # --- POST PROCESSING (YOLO Output Parsing) ---
        predictions = outputs[0]
        if predictions.ndim == 3 and predictions.shape[1] < predictions.shape[2]:
             predictions = predictions[0].transpose()
        elif predictions.ndim == 2:
             if predictions.shape[0] < predictions.shape[1]:
                 predictions = predictions.transpose()
        else:
             predictions = predictions[0]

        rows = predictions.shape[0]
        x_factor = image_width / INPUT_WIDTH
        y_factor = image_height / INPUT_HEIGHT

        class_ids = []
        confidences = []
        boxes = []

        for r in range(rows):
            row = predictions[r]
            if len(row) > 5: 
                confidence = row[4]
                classes_scores = row[5:]
            else: 
                classes_scores = row[4:]
                confidence = np.max(classes_scores)

            if confidence >= SCORE_THRESHOLD:
                class_id = np.argmax(classes_scores)
                if classes_scores[class_id] > SCORE_THRESHOLD:
                    cx, cy, w, h = row[0], row[1], row[2], row[3]
                    left = int((cx - w/2) * x_factor)
                    top = int((cy - h/2) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])

        indices = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD)
        
        msg = Float32MultiArray()
        target_data = [0.0, 0.0, 0.0] 

        if len(indices) > 0:
            # Logic to keep tracking the SAME box (Sticky Tracking)
            new_box = None
            if self.current_box is None:
                best_idx = indices[0]
                if isinstance(best_idx, (list, tuple, np.ndarray)): best_idx = best_idx[0]
                new_box = boxes[best_idx]
            else:
                prev_x, prev_y, prev_w, prev_h = self.current_box
                prev_center = (prev_x + prev_w//2, prev_y + prev_h//2)
                min_dist = float('inf')
                best_match_box = None
                
                for idx in indices:
                    if isinstance(idx, (list, tuple, np.ndarray)): idx = idx[0]
                    curr_box = boxes[idx]
                    cx, cy, cw, ch = curr_box
                    curr_center = (cx + cw//2, cy + ch//2)
                    dist = np.sqrt((prev_center[0] - curr_center[0])**2 + (prev_center[1] - curr_center[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_match_box = curr_box
                
                if min_dist < 200: new_box = best_match_box
                else: 
                    best_idx = indices[0]
                    if isinstance(best_idx, (list, tuple, np.ndarray)): best_idx = best_idx[0]
                    new_box = boxes[best_idx]

            if new_box is not None:
                self.current_box = new_box 
                x, y, w, h = new_box
                cx = x + (w // 2)
                
                # Error Calculation
                error_x = center_screen_x - cx 
                target_data = [1.0, float(error_x), float(w)]
            else:
                self.current_box = None
        else:
            self.current_box = None

        msg.data = target_data
        self.target_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.cam_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tracker = ObjectTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()