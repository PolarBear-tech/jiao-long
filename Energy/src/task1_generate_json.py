"""
检查矩形框，输出到json文件
"""
import cv2
import numpy as np
import json

def detect_R(video_path, output_video_path, output_json_path,
             R_AREA_MIN=180, R_AREA_MAX=240, ASPECT_TOL=15):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("无法打开视频")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 视频写入器（彩色输出，方便看框）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    kernel = np.ones((3, 3), np.uint8)
    results = []
    frame_idx = 0

    print(f"视频分辨率: {width}x{height}, FPS={fps}")
    print(f"R 面积范围: [{R_AREA_MIN}, {R_AREA_MAX}]")
    print(f"长宽误差容限: ±{ASPECT_TOL}")
    print("正在处理...")

    while True:
        ret, frame = cap.read()
        if not ret:
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

        # 可视化：在原图上画框
        vis = frame.copy() if len(frame.shape) == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # 收集 R 标
        R_centers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)

            # 条件1：面积在 [180, 240] 之间
            if area < R_AREA_MIN or area > R_AREA_MAX:
                continue

            # 获取外接矩形
            x, y, w, h = cv2.boundingRect(cnt)

            # 条件2：长宽基本一样，误差 ≤ 15
            if abs(w - h) > ASPECT_TOL:
                continue

            # 计算质心
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])

            # 画绿色矩形框
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 标中心红点
            cv2.circle(vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            # 写面积和尺寸文字
            cv2.putText(vis, f"R:{area:.0f} {w}x{h}", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            R_centers.append([round(cx, 1), round(cy, 1)])

        # 构造 JSON 输出
        output = {
            "frame": frame_idx,
            "targets": [
                {"type": "R", "center": R_centers[0] if len(R_centers) > 0 else []}
            ]
        }
        # 如果一帧有多个 R，全部输出
        if len(R_centers) > 1:
            output["targets"] = [{"type": "R", "center": c} for c in R_centers]

        results.append(output)

        # 写入视频帧
        writer.write(vis)
        frame_idx += 1

        if frame_idx % 30 == 0:
            print(f"已处理 {frame_idx} 帧")

    cap.release()
    writer.release()

    # 写 JSON
    with open(output_json_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f"完成！共 {frame_idx} 帧")
    print(f"带框视频: {output_video_path}")
    print(f"JSON 结果: {output_json_path}")

if __name__ == "__main__":
        video = "path/to/bitwize_mp4"
        out_vid = "path/to/detected_item_mp4"
        out_json = "path/to/output_json"
        detect_R(video, out_vid, out_json)