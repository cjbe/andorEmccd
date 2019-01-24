import andorEmccd
import time

cams = andorEmccd.initialise_all_cameras()

print("Cameras found: {}".format(len(cams)))
print("Serial numbers: {}".format(cams.keys()))

for sn, cam in cams.items():
    cam.set_temperature(-60)
    cam.set_exposure_time(1)
    def callback(_im, sn=sn):
        print("{}: got image".format(sn))
    cam.register_callback(callback)
    cam.start_acquisition()

try:
    while True:
        pass
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    for cam in cams.values():
        cam.close()
