import sensor
import time
import math

threshold_index = 0  # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
def zebraline(img):
    clock.tick()
    thresholds = [((0, 32, -117, 127, -31, 90))]
    for blob in img.find_blobs(
        [thresholds[0]],
        pixels_threshold=200,
        area_threshold=200,
        merge=True,
    ):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )
        zebra = blob.count()
        return zebra

def tri_reco(img):
    clock.tick()
    thresholds = [((28, 100, 51, -75, -49, 127))]
    for blob in img.find_blobs(
    [thresholds[0]],
    pixels_threshold=400,
    area_threshold=400,
    merge=True,
    ):
    # These values depend on the blob not being circular - otherwise they will be shaky.
     if blob.elongation() > 0.5:
        img.draw_edges(blob.min_corners(), color=(255, 0, 0))
        img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
        img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
    # These values are stable all the time.
    img.draw_rectangle(blob.rect())
    img.draw_cross(blob.cx(), blob.cy())
    # Note - the blob rotation is unique to 0-180 only.
    img.draw_keypoints(
        [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
    )
    tri = blob.count()
    return tri
while(True):
    img = sensor.snapshot()
    a = 0
    if zebraline(img)>0:
     a = 1
    b = 0
    if tri_reco(img)>3:
     b = 1
    print(a,b)
