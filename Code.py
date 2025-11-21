# Untitled - By: vaporious_yue - Thu Apr 18 2024

import sensor, image, time
from pyb import Servo
from random import randint
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)
uart = UART(3,115200,bits=8, parity=None, stop=1, timeout_char = 1000)
clock = time.clock()

# Servo pin set
pan_servo = Servo(1)
tilt_servo = Servo(2)

pan_servo.angle(0)
tilt_servo.angle(-90)


#threshold parameter set
threshold = (28, 100, 51, -75, -49, 127)


def calculate_blob_coverage(blobs, roi_area):
    total_blob_area = sum(blob.pixels() for blob in blobs)
    y = (total_blob_area / roi_area) * 100
    return y


def Lane_tracking(img):
    left = straight = right = 0
    blob1 = blob2 = blob3 = None
    G_threshold = (0,50)
    roi1=[(60, 150, 20, 40),
         (140, 150, 20, 40),
         (220, 150, 20, 40)]
    binary_img = img.binary([threshold])
    blob1 = binary_img.find_blobs([G_threshold],roi=roi1[0])
    blob2 = binary_img.find_blobs([G_threshold],roi=roi1[1])
    blob3 = binary_img.find_blobs([G_threshold],roi=roi1[2])

    roi_areas = [w * h for x, y, w, h in roi1]
    coverage1 = calculate_blob_coverage(blob1, roi_areas[0])
    coverage2 = calculate_blob_coverage(blob2, roi_areas[1])
    coverage3 = calculate_blob_coverage(blob3, roi_areas[2])

    # Determine directions based on blob coverage
    if coverage1 < 30 and coverage2 < 30 and coverage3 < 30:
            straight = 1
    if coverage3 >= 30:
            left = 1
    if coverage1 >= 30:
            right = 1

    for rec in roi1:
        img.draw_rectangle(rec, color=(255,0,0)) #绘制出roi区域

    return [left, straight, right]


def greenLight(img):
    stop = 0
    threshold_yelNet = (0, 100, -85, 37, -128, 20)
    G_threshold = (0,50)
    blob = None
    roi2=(80, 80, 140, 20)
    binary_img = img.binary([threshold_yelNet])
    blob = binary_img.find_blobs([G_threshold],roi=roi2)
    roi_areas = roi2[2]*roi2[3]
    coverage = calculate_blob_coverage(blob, roi_areas)
    if coverage > 50:
            stop = 1
    print(stop)
    img.draw_rectangle(roi2, color=(0,255,0))
    return stop






while(True):
    stop1 = 0
    left = straight = right = 0
    clock.tick()
    img = sensor.snapshot()
    [left, straight, right] = Lane_tracking(img)
    stop1 = greenLight(img)

    print(left, straight, right)

    # transmit message to mainboard
    binary_str = f"{stop1}{left}{straight}{right}"
    # 将二进制字符串转换为十进制值
    decimal_value = int(binary_str, 2)
    # 将十进制值转换为一个字节
    byte_value = bytes([decimal_value])
    # 通过UART发送字节
    uart.write(byte_value)
    print("Sent:", byte_value)










