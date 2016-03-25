"""An example of how to take a single image using the AndorEmccd class"""
import time
from andorEmccd import AndorEmccd


cam = AndorEmccd()

print("Camera initial temperature: {}".format(cam.get_temperature()))
cam.set_temperature(-60)

print("Waiting for camera to cool below -60C")
while cam.get_temperature() > -60:
    time.sleep(1)
print("Camera now at {}".format(cam.get_temperature()))

cam.set_shutter_open(True)
cam.set_exposure_time(0.1)
cam.start_acquisition(single=True)
cam.wait_for_acquisition()

print(cam.get_image())