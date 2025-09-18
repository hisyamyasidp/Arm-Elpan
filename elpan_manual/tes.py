from ultralytics import YOLO

# Load model pt
model = YOLO("/home/hisyam/Desktop/elpan_ws/src/elpan_manual/elpan_manual/botolvs.pt")

# Export ke onnx
model.export(format="onnx", opset=12)
