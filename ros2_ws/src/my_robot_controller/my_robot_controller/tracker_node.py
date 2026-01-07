#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
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
        self.model_path = os.path.join(package_share, 'config', 'mrc_yellow_cude_320.onnx')
    
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_info', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug', 10)

        self.tracking_enabled = False # Start Sleeping
        self.create_subscription(Bool, '/enable_tracking', self.enable_callback, 10)
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None        
        self.current_box = None        
        self.smooth_box = None      
        self.running = True            

        self.get_logger().info(f"Loading Model: {self.model_path}")
        try:
            sess_options = ort.SessionOptions()
            sess_options.intra_op_num_threads = 4 
            sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(self.model_path, sess_options, providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [o.name for o in self.session.get_outputs()]
            self.get_logger().info("Model Loaded!")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            exit()

        self.cam_thread = threading.Thread(target=self.camera_loop)
        self.cam_thread.start()
        
        self.timer = self.create_timer(0.01, self.ai_callback)

    def enable_callback(self, msg):
        self.tracking_enabled = msg.data
        if self.tracking_enabled:
            self.get_logger().info("TRACKER ENABLED: Hunting for Cube!")
        else:
            self.get_logger().info("TRACKER DISABLED: Sleeping...")
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

            if not self.tracking_enabled:
                time.sleep(0.1)
                continue

            with self.lock:
                self.latest_frame = frame.copy()

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
                cv2.rectangle(frame, (dx, dy), (dx + dw, dy + dh), (0, 255, 0), 3)
                cv2.putText(frame, "TARGET LOCKED", (dx, dy - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Show "SLEEPING" on the video feed if disabled
            if not self.tracking_enabled:
                 cv2.putText(frame, "AI SLEEPING", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_pub.publish(ros_image)
        
        cap.release()

    def ai_callback(self):
        if not self.tracking_enabled:
            return
        
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
            outputs = self.session.run(self.output_names, {self.input_name: img})
        except Exception: return

        predictions = outputs[0]
        # Handle Output Shape variations
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
        # Default: [0=Not Found, 0, 0]
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
                
                # --- CALCULATE ERROR ---
                # Negative = Object is to the Left (Robot turn Left)
                # Positive = Object is to the Right (Robot turn Right)
                error_x = center_screen_x - cx 
                
                # --- PACK DATA ---
                # [1.0 = FOUND, error_x, width]
                target_data = [1.0, float(error_x), float(w)]
            else:
                self.current_box = None
        else:
            self.current_box = None

        # Publish the data for Mission Controller to use
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