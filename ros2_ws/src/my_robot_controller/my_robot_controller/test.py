import onnxruntime as ort
import numpy as np
import os

# --- CONFIGURATION ---
MODEL_PATH = '/home/robocon/ros_baby/ros2_ws/src/my_robot_controller/config/platform.onnx'
INPUT_SIZE = 320  # Assuming 320x320 based on your previous messages

def test_model():
    print(f"Checking path: {MODEL_PATH}")
    if not os.path.exists(MODEL_PATH):
        print("❌ ERROR: File not found! Check the path.")
        return

    try:
        # 1. Load the Model
        print("Loading ONNX model...")
        session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
        print("✅ Model loaded successfully!")

        # 2. Get Input Details
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        print(f"   Input Name: {input_name}")
        print(f"   Expected Shape: {input_shape}")

        # 3. Create Dummy Input (Random Noise)
        # Standard YOLO input is (1, 3, Height, Width) normalized 0-1
        dummy_input = np.random.rand(1, 3, INPUT_SIZE, INPUT_SIZE).astype(np.float32)
        print("   Created dummy input tensor.")

        # 4. Run Inference
        print("Running inference...")
        outputs = session.run(None, {input_name: dummy_input})
        
        # 5. Check Output
        output_shape = outputs[0].shape
        print("✅ Inference Successful!")
        print(f"   Output Shape: {output_shape}")
        print("   (Rows = detections, Cols = x, y, w, h, conf, class_scores...)")

    except Exception as e:
        print(f"\n❌ FAILED with error:\n{e}")

if __name__ == "__main__":
    test_model()