import cv2
import numpy as np
import sys

def video_orange_binary(video_path, output_path=None,lower_orange=None, upper_orange=None):
    # 1. 打开视频（video_path 可以是本地文件路径，也可以是摄像头索引 0）
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print("无法打开视频，请检查路径")
        return

    # 2. 获取视频的基本信息（用于保存输出视频）
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 如果指定了输出路径，初始化视频写入器
    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height),isColor=False)  # 输出为灰度图像

    print("按 'q' 键退出播放")

    while True:
        # 3. 逐帧读取视频
        ret, frame = cap.read()
        if not ret:
            break  # 视频播放完毕

        # 4. 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 6. 生成二值掩码并提取橙色区域
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # 可选：形态学操作去除噪点
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # 7. 实时显示结果
        cv2.imshow("Original Video", frame)
        cv2.imshow("Orange Mask", mask)
        cv2.imshow("Orange Only", result)

        # 8. 如果开启了保存，写入当前帧
        if writer:
            writer.write(mask)  # 保存二值掩码

        # 9. 按 'q' 键退出（waitKey 的延迟设为 1ms 保证播放流畅）
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # 10. 释放资源
    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    print("处理完成！")

if __name__ == "__main__":
        #video_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy1.mkv"
        #output_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy1_orange_only.mp4"
        #video_orange_binary(video_path, output_path)
        video_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy2.mkv"
        output_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy2_orange_only.mp4"
        video_orange_binary(video_path, output_path,lower_orange= np.array([0, 169, 60]),upper_orange = np.array([132, 255, 255]))
        video_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy3.mkv"
        output_path = "/home/gufeng/27笔试/27笔试/down/Energy/energy3_orange_only.mp4"
        video_orange_binary(video_path, output_path,lower_orange= np.array([0, 124, 35]),upper_orange = np.array([179, 255, 255]))