import ctypes
import time
import platform
import os
import numpy as np

# Constants returned by Andor SDK
DRV_SUCCESS = 20002
DRV_NO_NEW_DATA = 20024
DRV_TEMPERATURE_OFF = 20034
DRV_P1INVALID = 20066
DRV_P2INVALID = 20067
DRV_P3INVALID = 20068
DRV_P4INVALID = 20069
DRV_ACQUIRING = 20072
DRV_NOT_INITIALIZED = 20075

# Constants for SetTrigger()
TRIGGER_INTERNAL = 0
TRIGGER_EXTERNAL = 1

# Constants for SetReadMode()
READMODE_SINGLETRACK = 3
READMODE_IMAGE = 4

# Constants for SetAcquitisionMode()
ACQUISITION_SINGLE = 2
ACQUISITION_RUN_TILL_ABORT = 5


class AndorEmccd:
    """A dumb SDK wrapper for Andor iXon EMCCD cameras.
    Note that almost all camera parameters can only be changed when the camera is not acquiring"""
    def __init__(self, leave_camera_warm=False):
        self.leave_camera_warm = leave_camera_warm

        if platform.system() == "Windows":
            os.environ['PATH'] = "C:\Program Files\Andor SOLIS\Drivers" + ';' + os.environ['PATH']
            if platform.architecture()[0] == "32bit":
                self.dll = ctypes.WinDLL("atmcd32d.dll")
            else:
                self.dll = ctypes.WinDLL("atmcd64d.dll")
        elif platform.system() == "Linux":
            self.dll = ctypes.cdll.LoadLibrary("/usr/local/lib/libandor.so")
        else:
            raise Exception("Unsupported operating system")

        ret = self.dll.Initialize("")
        if ret != DRV_SUCCESS:
            raise Exception("Could not initialise camera")

        self.dll.SetCoolerMode(int(not self.leave_camera_warm))

        # We are responsible adults - we can handle the full EM gain!
        self.dll.SetEMAdvanced(int(1))
        # Set 'real EM gain' mode
        self.dll.SetEMGainMode(int(3))

        horiz = ctypes.c_int()
        vert = ctypes.c_int()
        self.dll.GetDetector(ctypes.byref(horiz), ctypes.byref(vert))
        self.ccdWidth = horiz.value
        self.ccdHeight = vert.value

        # Set the default ROI to the full sensor
        self.set_image_region(0, self.ccdWidth-1, 0, self.ccdHeight-1, hBin=1, vBin=1)

    def __del__(self):
        if self.leave_camera_warm:
            self.set_temperature(10)
            print("Waiting for camera to warm up ...")
            while self.get_temperature() < -20:
                # Wait for camera to warm up into the safe temperature range of >-20
                time.sleep(1)
            print("Camera now at T={}".format(self.get_temperature()))
        self.dll.ShutDown()

    def set_trigger_mode(self, trig_mode):
        """Set the trigger mode between internal and external"""
        ret = self.dll.SetTriggerMode(int(trig_mode))
        if ret != DRV_SUCCESS:
            raise Exception()

    def set_temperature(self, temp):
        """Set the temperature setpoint to temp deg C"""
        ret = self.dll.CoolerON()
        if ret != DRV_SUCCESS:
            raise Exception()
        ret = self.dll.SetTemperature(int(temp))
        if ret != DRV_SUCCESS:
            raise Exception()

    def get_temperature(self):
        """Returns the current camera temperature in deg C, or None if cooler is off"""
        T = ctypes.c_int()
        ret = self.dll.GetTemperature(ctypes.byref(T))
        if ret == DRV_TEMPERATURE_OFF:
            return None
        elif ret != DRV_SUCCESS:
            raise Exception()
        return T.value

    def set_image_region(hStart, hEnd, vStart, vEnd, hBin=1, vBin=1):
        """Set the CCD region to read out and the horizontal and vertical binning"""
        self.roiWidth = (hEnd-hStart) / hBin
        self.roiHeight = (vEnd-vStart) / vBin

        self.dll.SetReadMode(4) # Image
        ret = self.dll.SetImage(int(hBin), int(vBin), int(hStart), int(hEnd), int(vStart), int(vEnd))
        if ret != DRV_SUCCESS:
            raise Exception()

    def set_em_gain(self, gain):
        """Set the EM gain multiplication factor"""
        ret = self.dll.SetEMCCDGain(int(gain))
        if ret == DRV_P1INVALID:
            raise Exception("Invalid EM Gain value")
        if ret != DRV_SUCCESS:
            raise Exception()

    def set_exposure_time(self, time):
        """Set the CCD exposure time in seconds"""
        ret = self.dll.SetExposureTime(ctypes.c_float(time))
        if ret != DRV_SUCCESS:
            raise Exception()

    def set_shutter_open(self, shutter_open):
        """Open / close the camera shutter"""
        ret = self.dll.SetShutter(1, int(1 if shutter_open else 2), 0, 0)
        if ret != DRV_SUCCESS:
            raise Exception()

    def start_acquisition(self, single=False):
        """Start a single or repeated acquisition. If single=False the acquisition is repeated as fast as possible, (or 
        on every trigger, if in 'external trigger' mode) until stop_acquisition() is called."""
        if single:
            mode = ACQUISITION_SINGLE
        else:
            mode = ACQUISITION_RUN_TILL_ABORT

        ret = self.dll.SetAcquisitionMode(mode)
        if ret != DRV_SUCCESS:
            raise Exception()
        ret = self.dll.SetAcquisition()
        if ret != DRV_SUCCESS:
            raise Exception()

    def stop_acquisition(self):
        """Stop a repeated acquisition"""
        ret = self.dll.AbortAcquisition()
        if ret != DRV_SUCCESS:
            raise Exception()

    def wait_for_acquisition(self):
        """Wait for a new image to become available"""
        ret = self.dll.WaitForAcquisition()
        if ret != DRV_SUCCESS:
            raise Exception()

    def get_image(self):
        """Returns the oldest image in the buffer as a numpy array, or None if no new images"""
        imSize = self.roiWidth*self.roiHeight
        buf = (ctypes.c_int * imSize)()
        ret = self.dll.GetOldestImage(buf, ctypes.c_ulong(imSize))
        if ret == DRV_NO_NEW_DATA:
            return None
        elif ret == DRV_P2INVALID:
            raise Exception("Internal error: Array size is incorrect")
        elif ret != DRV_SUCCESS:
            raise Exception()

        im = np.frombuffer(buf, dtype=np.int32)
        im = im.reshape(self.roiWidth, self.roiHeight)
        im = np.transpose(im)
        return im