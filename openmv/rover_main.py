# Rover OpenMV 主程序
# 统一通信协议，支持所有任务检测

import sensor, image, time, math, struct
from pyb import UART, LED
from image import SEARCH_EX, SEARCH_DS

#==============================================================================
# 通信协议定义 (与STM32端一致)
#==============================================================================
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55

# 命令码
CMD_LANE = 0x01      # 车道检测
CMD_COLOR = 0x02     # 颜色检测
CMD_ARROW = 0x03     # 箭头识别
CMD_LINE = 0x04      # 停止线检测
CMD_TARGET = 0x05    # 目标位置
CMD_BASKET = 0x06    # 篮筐检测

# 箭头方向
ARROW_NONE = 0
ARROW_LEFT = 1
ARROW_STRAIGHT = 2
ARROW_RIGHT = 3

# 颜色
COLOR_NONE = 0
COLOR_RED = 1
COLOR_GREEN = 2
COLOR_BLUE = 3
COLOR_YELLOW = 4

#==============================================================================
# 硬件初始化
#==============================================================================
# UART初始化 (与STM32 UART2对接)
uart = UART(3, 115200, bits=8, parity=None, stop=1, timeout_char=1000)

# 传感器初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_contrast(1)
sensor.set_gainceiling(16)

clock = time.clock()
led = LED(1)  # 红色LED

#==============================================================================
# 阈值参数配置
#==============================================================================
# 车道颜色阈值 (LAB颜色空间)
LANE_THRESHOLD = (28, 100, 51, -75, -49, 127)

# 停止线阈值 (黄色/红色线)
STOP_LINE_THRESHOLD = (0, 100, -85, 37, -128, 20)

# 颜色阈值
RED_THRESHOLD = (30, 100, 15, 127, 15, 127)
GREEN_THRESHOLD = (30, 100, -64, -8, -32, 32)
BLUE_THRESHOLD = (0, 30, 0, 30, -128, -20)
YELLOW_THRESHOLD = (30, 100, -10, 30, 30, 127)

# 斑马线阈值
ZEBRA_THRESHOLD = (0, 32, -117, 127, -31, 90)

#==============================================================================
# ROI区域定义
#==============================================================================
# 车道检测ROI (左、中、右三个区域)
LANE_ROI = [
    (60, 150, 20, 40),   # 左
    (140, 150, 20, 40),  # 中
    (220, 150, 20, 40)   # 右
]

# 停止线检测ROI
STOP_LINE_ROI = (80, 80, 160, 30)

# 箭头检测ROI
ARROW_ROI = (80, 60, 160, 120)

#==============================================================================
# 模板加载 (箭头识别)
#==============================================================================
try:
    template_left = image.Image("/left_arrow.pgm")
    template_straight = image.Image("/straight_arrow.pgm")
    template_right = image.Image("/right_arrow.pgm")
    TEMPLATES_LOADED = True
except:
    TEMPLATES_LOADED = False
    print("Warning: Arrow templates not found")

#==============================================================================
# 通信函数
#==============================================================================
def send_frame(cmd, data):
    """发送数据帧"""
    frame = bytearray()
    frame.append(FRAME_HEADER)
    frame.append(cmd)
    frame.append(len(data))

    checksum = FRAME_HEADER + cmd + len(data)
    for b in data:
        frame.append(b)
        checksum += b

    frame.append(checksum & 0xFF)
    uart.write(frame)

def send_lane_data(offset, angle):
    """发送车道数据"""
    # offset和angle为int16
    data = struct.pack('<hh', offset, angle)
    send_frame(CMD_LANE, data)

def send_color(color):
    """发送颜色检测结果"""
    send_frame(CMD_COLOR, bytes([color]))

def send_arrow(direction):
    """发送箭头识别结果"""
    send_frame(CMD_ARROW, bytes([direction]))

def send_stop_line(detected, distance):
    """发送停止线检测结果"""
    data = struct.pack('<BH', detected, distance)
    send_frame(CMD_LINE, data)

def send_target(x, y):
    """发送目标位置"""
    data = struct.pack('<hh', x, y)
    send_frame(CMD_TARGET, data)

#==============================================================================
# 检测函数
#==============================================================================
def calculate_blob_coverage(blobs, roi_area):
    """计算色块覆盖率"""
    if not blobs:
        return 0
    total_area = sum(blob.pixels() for blob in blobs)
    return (total_area / roi_area) * 100

def detect_lane(img):
    """
    车道检测
    返回: (offset, angle) - 偏移量和角度
    """
    # 二值化处理
    binary_img = img.copy().binary([LANE_THRESHOLD])

    # 检测三个区域的覆盖率
    G_threshold = (0, 50)
    roi_areas = [roi[2] * roi[3] for roi in LANE_ROI]

    coverages = []
    for i, roi in enumerate(LANE_ROI):
        blobs = binary_img.find_blobs([G_threshold], roi=roi)
        coverage = calculate_blob_coverage(blobs, roi_areas[i])
        coverages.append(coverage)
        img.draw_rectangle(roi, color=(255, 0, 0))

    # 计算偏移量 (-160 到 160)
    left_cov, mid_cov, right_cov = coverages

    if left_cov >= 30 and right_cov < 30:
        offset = -80  # 偏右，需要左转
    elif right_cov >= 30 and left_cov < 30:
        offset = 80   # 偏左，需要右转
    elif left_cov >= 30 and right_cov >= 30:
        offset = 0    # 在中间
    else:
        offset = 0    # 直行

    # 使用线检测计算角度
    angle = 0
    lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)
    if lines:
        # 取最长的线计算角度
        best_line = max(lines, key=lambda l: l.length())
        angle = best_line.theta() - 90  # 转换为偏差角度
        img.draw_line(best_line.line(), color=(0, 255, 0))

    return int(offset), int(angle)

def detect_stop_line(img):
    """
    停止线检测
    返回: (detected, distance)
    """
    binary_img = img.copy().binary([STOP_LINE_THRESHOLD])
    G_threshold = (0, 50)

    blobs = binary_img.find_blobs([G_threshold], roi=STOP_LINE_ROI,
                                   pixels_threshold=200, merge=True)

    img.draw_rectangle(STOP_LINE_ROI, color=(0, 255, 0))

    if blobs:
        # 找最大的blob
        largest = max(blobs, key=lambda b: b.pixels())
        coverage = (largest.pixels() / (STOP_LINE_ROI[2] * STOP_LINE_ROI[3])) * 100

        if coverage > 40:
            # 计算距离 (基于y坐标)
            distance = 240 - largest.cy()
            img.draw_rectangle(largest.rect(), color=(255, 255, 0))
            return 1, distance

    return 0, 0

def detect_arrow(img):
    """
    箭头识别
    返回: 方向 (ARROW_LEFT/STRAIGHT/RIGHT/NONE)
    """
    if not TEMPLATES_LOADED:
        return ARROW_NONE

    # 转灰度图进行模板匹配
    gray_img = img.copy().to_grayscale()

    # 匹配三个方向的箭头
    r_left = gray_img.find_template(template_left, 0.65, step=4,
                                     search=SEARCH_EX, roi=ARROW_ROI)
    r_straight = gray_img.find_template(template_straight, 0.65, step=4,
                                         search=SEARCH_EX, roi=ARROW_ROI)
    r_right = gray_img.find_template(template_right, 0.65, step=4,
                                      search=SEARCH_EX, roi=ARROW_ROI)

    if r_left:
        img.draw_rectangle(r_left, color=(255, 0, 0))
        return ARROW_LEFT
    elif r_straight:
        img.draw_rectangle(r_straight, color=(0, 255, 0))
        return ARROW_STRAIGHT
    elif r_right:
        img.draw_rectangle(r_right, color=(0, 0, 255))
        return ARROW_RIGHT

    return ARROW_NONE

def detect_color(img):
    """
    颜色检测
    返回: 颜色代码
    """
    # 检测各种颜色
    thresholds = [
        (RED_THRESHOLD, COLOR_RED),
        (GREEN_THRESHOLD, COLOR_GREEN),
        (BLUE_THRESHOLD, COLOR_BLUE),
        (YELLOW_THRESHOLD, COLOR_YELLOW)
    ]

    for thresh, color_code in thresholds:
        blobs = img.find_blobs([thresh], pixels_threshold=500,
                               area_threshold=500, merge=True)
        if blobs:
            largest = max(blobs, key=lambda b: b.pixels())
            if largest.pixels() > 1000:
                img.draw_rectangle(largest.rect(), color=(255, 255, 255))
                return color_code

    return COLOR_NONE

def detect_target(img, threshold):
    """
    目标检测 (用于门框、篮筐等)
    返回: (found, x, y)
    """
    blobs = img.find_blobs([threshold], pixels_threshold=500,
                           area_threshold=500, merge=True)

    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(largest.rect(), color=(255, 0, 255))
        img.draw_cross(largest.cx(), largest.cy(), color=(255, 0, 255))
        return True, largest.cx(), largest.cy()

    return False, 0, 0

#==============================================================================
# 运行模式
#==============================================================================
# 运行模式定义
MODE_LANE_FOLLOW = 0    # 车道跟随
MODE_ARROW_DETECT = 1   # 箭头检测
MODE_COLOR_DETECT = 2   # 颜色检测
MODE_TARGET_FIND = 3    # 目标查找

current_mode = MODE_LANE_FOLLOW

def process_command():
    """处理来自STM32的命令"""
    global current_mode
    if uart.any():
        cmd = uart.read(1)
        if cmd:
            cmd_byte = cmd[0]
            if cmd_byte == 0x10:
                current_mode = MODE_LANE_FOLLOW
            elif cmd_byte == 0x11:
                current_mode = MODE_ARROW_DETECT
            elif cmd_byte == 0x12:
                current_mode = MODE_COLOR_DETECT
            elif cmd_byte == 0x13:
                current_mode = MODE_TARGET_FIND

#==============================================================================
# 主循环
#==============================================================================
print("Rover OpenMV Ready")
led.on()
time.sleep_ms(500)
led.off()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 处理STM32命令
    process_command()

    # 根据模式执行检测
    if current_mode == MODE_LANE_FOLLOW:
        # 车道跟随模式
        offset, angle = detect_lane(img)
        send_lane_data(offset, angle)

        # 同时检测停止线
        stop_detected, stop_dist = detect_stop_line(img)
        if stop_detected:
            send_stop_line(stop_detected, stop_dist)

    elif current_mode == MODE_ARROW_DETECT:
        # 箭头检测模式
        arrow = detect_arrow(img)
        send_arrow(arrow)

    elif current_mode == MODE_COLOR_DETECT:
        # 颜色检测模式
        color = detect_color(img)
        send_color(color)

    elif current_mode == MODE_TARGET_FIND:
        # 目标查找模式 (用于门框/篮筐)
        found, x, y = detect_target(img, LANE_THRESHOLD)
        if found:
            send_target(x - 160, y - 120)  # 转换为中心偏移

    # LED闪烁表示运行
    if clock.fps() > 10:
        led.toggle()

    # 调试输出
    # print("FPS:", clock.fps())
