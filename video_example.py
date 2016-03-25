"""An example of how to take a continuous stream of images using the AndorEmccd class"""
import time
import matplotlib.pyplot as plt
from andorEmccd import AndorEmccd
import time

cam = AndorEmccd()

print("Camera initial temperature: {}".format(cam.get_temperature()))
cam.set_temperature(-80)

print("Waiting for camera to cool below -60C")
while cam.get_temperature() > -60:
    time.sleep(1)
print("Camera now at {}".format(cam.get_temperature()))

cam.set_image_region(1,512, 250,300)
cam.set_shutter_open(True)
cam.set_vertical_shift_speed(0.3)
cam.set_exposure_time(0.001)

cam.start_acquisition()


n_acquired = 0
t_start = time.time()
while n_acquired < 1000:
    imVec = cam.get_all_images()
    if imVec is None:
        continue
    n_acquired += len(imVec)
print("n_acquired = {}. Frame rate = {} /s".format(n_acquired, n_acquired/(time.time()-t_start)))
