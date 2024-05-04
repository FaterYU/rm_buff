# rm_buff
RoboMaster buff detect and predict

## 模型训练配置

### 1. 数据集

使用西交利物浦GMaster战队开源数据集：[YOLO-of-RoboMaster-Keypoints-Detection-2023](https://github.com/zRzRzRzRzRzRzR/YOLO-of-RoboMaster-Keypoints-Detection-2023)

转化为 `yolov8 keypoint` 格式

```bash
python ./scripts/split_data.py --data ./datasets/buff_data --ratio 0.8 --output ./datasets/buff_format
```

### 2. 训练

```bash
yolo pose train data=buff.yaml model=yolov8n-pose.pt epochs=200 batch=32 imgsz=640 iou=0.7 max_det=10 kobj=10 rect=True name=buff
```

### 3. pt -> onnx

```bash
yolo export model=models/best.pt format=onnx dynamic=False half=True simplify=True opset=13
```

### 4. onnx -> ov

```bash
mo --input_model ./models/best.onnx --output_dir ./models
```

### 5. 模型量化

```bash
python ./scripts/ov_quantize.py
```
