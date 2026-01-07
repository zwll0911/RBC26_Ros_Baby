import onnx
from onnxruntime.quantization import quantize_dynamic, QuantType

# --- USE FULL PATHS HERE ---
model_fp32 = '/home/robocon/Desktop/mrc_yellow_cube.onnx'        # Input
model_int8 = '/home/robocon/Desktop/mrc_yellow_cube_int8.onnx'   # Output

print(f"Quantizing {model_fp32}...")

quantize_dynamic(
    model_input=model_fp32,
    model_output=model_int8,
    weight_type=QuantType.QUInt8
)

print(f"Done! Saved as {model_int8}")