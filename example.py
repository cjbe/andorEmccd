"""An example of how to take a single image using the AndorEmccd class"""
import time
from andorEmccd import AndorEmccd

cam = AndorEmccd()

cam.set_temperature(-80)

cam.set_shutter_open(True)
cam.set_exposure_time(0.1)
cam.start_acquisition(single=True)

im = cam.wait_for_image()
print(im)