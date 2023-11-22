import cv2
from ultralytics import YOLO

# 加载YOLOv8模型
model = YOLO('best_100b.pt')

# 打开视频文件
video_path = 'l1_rune.mp4'
cap = cv2.VideoCapture(video_path)

fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 指定视频视频编解码器格式
fps = cap.get(cv2.CAP_PROP_FPS) #帧率
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) #自动获取视频大小
out = cv2.VideoWriter('output_100b.avi', fourcc, fps, size)  #opencv好像只能导出avi格式

# 遍历视频帧
while cap.isOpened():
    # 从视频中读取一帧
    success, frame = cap.read()

    if success:
        # 在该帧上运行YOLOv8推理
        results = model(frame)

        # 在帧上可视化结果
        annotated_frame = results[0].plot()
        
        # 写入视频文件
        out.write(annotated_frame)

        # # 显示带注释的帧
        # cv2.imshow("YOLOv8推理", annotated_frame)

        # # 如果按下'q'则中断循环
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     break
    else:
        # 如果视频结束则中断循环
        break

# 释放视频捕获对象并关闭显示窗口
cap.release()
cv2.destroyAllWindows()
