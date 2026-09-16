import cv2
import numpy as np
import sys

def nothing(x):
    """滑动条回调函数（OpenCV 要求必须有）"""
    pass

def interactive_color_picker(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("无法打开视频，请检查路径")
        return

    # 只创建一个滑动条窗口
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("HMin", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("SMin", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("VMin", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("HMax", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("SMax", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("VMax", "Trackbars", 255, 255, nothing)

    # 创建一个显示结果的窗口
    cv2.namedWindow("Result")

    print("拖动滑动条调整颜色范围，按 'q' 键退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # 实时读取滑动条的值
        h_min = cv2.getTrackbarPos("HMin", "Trackbars")
        s_min = cv2.getTrackbarPos("SMin", "Trackbars")
        v_min = cv2.getTrackbarPos("VMin", "Trackbars")
        h_max = cv2.getTrackbarPos("HMax", "Trackbars")
        s_max = cv2.getTrackbarPos("SMax", "Trackbars")
        v_max = cv2.getTrackbarPos("VMax", "Trackbars")

        # HSV 转换与阈值分割
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学去噪
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        # 只显示两个窗口：滑动条窗口 + 结果窗口
        cv2.imshow("Trackbars", np.zeros((100, 400, 3), dtype=np.uint8))  # 滑动条载体窗口
        cv2.imshow("Result", result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # 退出时打印当前阈值
    print(f"\n当前阈值范围：")
    print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
    print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
        interactive_color_picker("/home/gufeng/27笔试/27笔试/down/Energy/energy1.mkv")