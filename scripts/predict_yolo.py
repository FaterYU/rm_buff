import cv2
from ultralytics import YOLO

model_path = './models/armor-5.pt'
img_path = './input/sample.jpg'
img = cv2.imread(img_path)
img = cv2.resize(img, (640, 480))

model = YOLO(model_path)

results = model(img)

annotated_frame = results[0].plot()

print(results[0].speed)

# 定义字体和字体大小
font = cv2.FONT_HERSHEY_SIMPLEX
font_size = 0.6
font_thickness = 1

# 循环遍历速度字典并在图像上标注文本
for i, (key, value) in enumerate(results[0].speed.items()):
    text = f'{key}: {value:.2f} ms'
    y_position = 30 + i * 30  # 调整文本的垂直位置
    cv2.putText(annotated_frame, text, (10, y_position), font,
                font_size, (0, 255, 0), font_thickness, cv2.LINE_AA)


cv2.imshow("YOLOv8推理", annotated_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
