import time
from andorEmccd import AndorEmccd

cam = AndorEmccd()

cam.set_shutter_open(False)
cam.set_exposure_time(0.1)
cam.set_em_gain(1)

def callback(im):
    print("Acq!")

cam.register_callback(callback)


cam.start_acquisition()

time.sleep(2)

print("Bored now")
cam.stop_acquisition()
