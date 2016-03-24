"""An example of how to take a single image using the AndorEmccd class"""
import time
from andorEmccd import AndorEmccd


cam = AndorEmccd()

print("Camera initial temperature: {}".format(cam.get_temperature()))
cam.set_temperature(-60)

print("Waiting for camera to cool below -50C")
while cam.get_temperature() > -50:
    time.sleep(1)
print("Done")

cam.set_shutter_open(True)
