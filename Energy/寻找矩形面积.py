import cv2
import numpy as np
import sys

def debug_area(video_path, min_area=100):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("无法打开视频")
        return

    kernel = np.ones((3, 3), np.uint8)
    frame_idx = 0

    print("=" * 60)
    print("调试模式：按 'n' 下一帧，按 'q' 退出")
    print("=" * 60)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("视频结束")
            break

        # 转灰度 + 二值化
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        _, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 打印当前帧信息
        print(f"\n--- 帧 {frame_idx} ---")
        valid_count = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            x, y, w, h = cv2.boundingRect(cnt)
            print(f"  轮廓{i}: 面积={area:.0f}, 中心=({cx:.1f}, {cy:.1f}), 矩形=({x},{y},{w}x{h})")
            valid_count += 1

        if valid_count == 0:
            print("  （无有效轮廓）")

        # 可视化：画绿色矩形框 + 面积文字
        vis = frame.copy() if len(frame.shape) == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(vis, f"{area:.0f}", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Debug", vis)
        key = cv2.waitKey(0) & 0xFF  # 按任意键继续下一帧
        if key == ord('q'):
            break
        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n共处理 {frame_idx} 帧")

if __name__ == "__main__":
    video = "/home/gufeng/谷丰525712910059/energy1_orange_only.mp4"
    debug_area(video)