# Untitled - By: vaporious_yue - Thu Apr 18 2024

import sensor, image, time
from pyb import Servo
from random import randint
from pyb import UART
from image import SEARCH_EX

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)
uart = UART(3,115200,bits=8, parity=None, stop=1, timeout_char = 1000)
clock = time.clock()
sensor.set_contrast(1)
sensor.set_gainceiling(16)
template_right = image.Image("/1.pgm")
template_go = image.Image("/2.pgm")
template_left= image.Image("/3.pgm")

def markerrecognition(img):
    L=S=R=0
    # find_template(template, threshold, [roi, step, search])
    # ROI: The region of interest tuple (x, y, w, h).
    # Step: The loop step used (y+=step, x+=step) use a bigger step to make it faster.
    # Search is either image.SEARCH_EX for exhaustive search or image.SEARCH_DS for diamond search
    #
    # Note1: ROI has to be smaller than the image and bigger than the template.
    # Note2: In diamond search, step and ROI are both ignored.
    r1 = img.find_template(
        template_right, 0.70, step=4, search=SEARCH_EX
    )  # , roi=(10, 0, 60, 60))
    if r1:
        img.draw_rectangle(r1)
        R=1
    r2= img.find_template(
        template_go, 0.70, step=4, search=SEARCH_EX
    )  # , roi=(10, 0, 60, 60))
    if r2:
        img.draw_rectangle(r2)
        S=1
    r3 = img.find_template(
        template_left, 0.70, step=4, search=SEARCH_EX
        )  # , roi=(10, 0, 60, 60))
    if r3:
        img.draw_rectangle(r3)
        L=1
    return [L,S,R]

while(True):
    img = sensor.snapshot()
    [L,S,R] = markerrecognition (img)
    print(L,S,R)
