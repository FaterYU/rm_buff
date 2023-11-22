import openvino as ov

from ultralytics import YOLO

model = YOLO('best_100b.pt')

model.export(format="openvino", dynamic=False, half=True)

