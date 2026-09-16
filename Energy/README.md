# Task 1 思路

0. 在[py_0](./src/color_picker.py)中实验得到最合适的HSV范围
1. 读取mkv的文件，每一帧转成hsv，形成灰度图像，二值化，使用[py_1](./src/process_mkv2mp4.py)，形成[mp4_1](./mp4/energy1_orange_only.mp4)
2. [py_2](./src/find_rect_area.py)读取二值化后的mp4_1文件，检测R所在的area，使用绿色矩形框出，输出到[mp4_2](./mp4/energy1_R_detected.mp4)
3. [py_3](./src/task1_generate_json.py)通过mp4_1和mp4_2准确定位，输出到[json_1](./1.json)