"""
检查R和目标靶的矩形范围，并输出mp4的视频

这里有一个区分的问题，R标是比较杂乱的，目标靶是比较圆的，
而且中间有机械臂也会发光，是矩形的，区分这三点是重点
"""

# 和task1_find_rect_area的逻辑相似

import cv2 as cv
import numpy as np
import json

def find_exact_R(R_pos_list, fan_pos):
    if len(R_pos_list) == 0:
        return None
    elif fan_pos is None:
        return R_pos_list[0]
    (fan_x, fan_y) = fan_pos
    res = None
    dis = None
    for (x, y) in R_pos_list:
        temp = ((fan_x - x)**2+(fan_y-y)**2)**0.5
        if dis is None:
            res = (x, y)
            dis = temp
        elif temp < dis:
            res = (x, y)
            dis = temp
        else:
            continue
    return res if res[0] != -1 else None
        

def get_output(frame_idx, R_pos, fan_pos):
    output = {
        "frame": frame_idx,
        "targets": [
            {"type": "R", "center": R_pos if R_pos is not None else []},
            {"type": "fan", "center": fan_pos if fan_pos is not None else []}
        ]
    }
    return output


def debug_area(video_path, output_json,min_area=50, min_cuir=0.7, R_area_max=400, arm_aspect_thresh=2):
    src = cv.VideoCapture(video_path)
    if not src.isOpened():
        print(f"open {video_path} failed.")
        return

    print("=" * 60)
    print("调试模式：按 'q' 退出")
    print("=" * 60)
    # 准备一下噪点的kernel
    kernel = np.ones((3, 3), np.uint8)
    # 代表视频帧索引
    frame_index = 0
    # 展示标记好的帧的window的窗口名
    RESULT_WINDOW = "result"

    res_json = []
    while True:
        ret, frame = src.read()
        if not ret:
            print("EOF")
            break

        # 转成灰度图像
        if len(frame.shape) == 3:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        else:
            gray = frame  # -> 已经是灰度图像，直接赋值就好了
        # 二值化
        _, mask = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
        # 去掉白色噪点
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        # 补充轮廓
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        # 最终绘制的图像
        # 使用mask 而不是frame的原因是我们希望draw中已经去掉了不必要的噪点
        draw = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

        # 检测一下轮廓
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 记录结果
        container = {"R": 0, "fan": 0,"arm":0, "unknown": 0}

        fan_pos = None
        R_pos_list = []
        print(f"\n --- 帧 {frame_index} ---")
        for i, cnt in enumerate(contours):
            area = cv.contourArea(cnt)
            # 过滤掉太小的噪点
            if area < min_area:
                continue
            peri = cv.arcLength(cnt, True)

            # 圆形度
            circularity = 4 * np.pi * area / (peri ** 2) if peri > 0 else 0

            x, y, w, h = cv.boundingRect(cnt)

            box_long = max(w, h)
            box_short = min(w, h)
            box_aspect = box_long / box_short if box_short>0 else 0

            if box_aspect > arm_aspect_thresh:
                # arm
                cv.rectangle(draw, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(draw, f"arm {area:.0f}", (x, y - 5),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                print(f"find a arm at position {i}: center({x + w // 2},{y + h //2}), area({area})")
                fan_pos = (x + w // 2, y + h // 2)
                container["arm"] += 1
            # 细分一下
            if circularity > min_cuir:
                # fan
                cv.rectangle(draw, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(draw, f"fan {area:.0f}", (x, y - 5),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                print(f"find a fan at position {i}: center({x + w // 2},{y + h //2}), area({area})")
                container["fan"] += 1
            elif area < R_area_max:
                cv.rectangle(draw, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(draw, f"R {area:.0f}", (x, y - 5),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                print(f"find a R at position {i}: center({x + w // 2},{y + h //2}), area({area})")
                R_pos_list.append((x + w //2, y + h // 2))
                container["R"] += 1
            else:
                container["unknown"] += 1
                
            if sum(container.values()) == 0:
                print("nothing found.")
            # 输出一下标记好的图片
            cv.imshow(RESULT_WINDOW, draw)

        res = find_exact_R(R_pos_list, fan_pos)
        if res:
            print(f"R excatly at ({res[0]},{res[1]})")
        else:
            print("no R")
        output = get_output(frame_index, res, fan_pos)

        res_json.append(output)
        key = cv.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        frame_index += 1

    src.release()
    cv.destroyAllWindows()
    print(f"\nprecessed cnt: {frame_index}")
    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(res_json, f, indent=2, ensure_ascii=False)



if __name__ == "__main__":
    
    video = "/home/gufeng/谷丰525712910059/Energy/mp4/energy2_orange_only.mp4"
    output_json = "./Energy/2.json"
    debug_area(video, output_json)
