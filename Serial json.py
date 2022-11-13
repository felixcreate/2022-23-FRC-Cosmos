import sensor, image, time, math, ujson, usys
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout

uart = UART(3, 115200, timeout_char=20)
uart.init(115200, bits=8, parity=1, stop=1, timeout_char=20)
clock = time.clock()

# f_x is the x focal length of the camera. It should be equal to the lens focal length in mm
# divided by the x sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# f_y is the y focal length of the camera. It should be equal to the lens focal length in mm
# divided by the y sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# c_x is the image x center position in pixels.
# c_y is the image y center position in pixels.

f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5

while(True):
    clock.tick()
    img = sensor.snapshot()
    tag_payload = []
    for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
        taginfo = {}
        taginfo["id"] = tag.id()
        taginfo["x"] = tag.x_translation()
        taginfo["y"] = tag.y_translation()
        taginfo["z"] = tag.z_translation()
        taginfo["rx"] = tag.x_rotation()
        taginfo["ry"] = tag.y_rotation()
        taginfo["rz"] = tag.z_rotation()
        taginfo["confidence"] = tag.decision_margin()
        tag_payload.append(taginfo)
    if(uart.any()):
        uart.read(uart.any())
        uart.write(ujson.dumps(tag_payload))
    print(clock.fps())
