import ctypes
import time
import platform
import os
import numpy as np

# Constants returned by Andor SDK
DRV_SUCCESS = 20002
DRV_NO_NEW_DATA = 20024
DRV_TEMPERATURE_OFF = 20034
DRV_TEMPERATURE_NOT_REACHED = 20037
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

        # Find the available gains, shift speeds, etc
        self._get_preamp_gains()
        self._get_vertical_shift_speeds()
        self._get_horizontal_shift_speeds()

        horiz = ctypes.c_int()
        vert = ctypes.c_int()
        self.dll.GetDetector(ctypes.byref(horiz), ctypes.byref(vert))
        self.ccdWidth = horiz.value
        self.ccdHeight = vert.value

        # Set the default ROI to the full sensor
        self.set_image_region(1, self.ccdWidth, 1, self.ccdHeight, hBin=1, vBin=1)

        # Sensible defaults
        self.set_shutter_open(True)
        self.set_trigger_mode(TRIGGER_INTERNAL)

    def __del__(self):
        if self.leave_camera_warm:
            self.set_temperature(10)
            print("Waiting for camera to warm up ...")
            while self.get_temperature() < -20:
                # Wait for camera to warm up into the safe temperature range of >-20
                time.sleep(1)
            print("Camera now at T={}".format(self.get_temperature()))
        self.dll.ShutDown()

    def _get_preamp_gains(self):
        tmp = ctypes.c_int()
        self.dll.GetNumberPreAmpGains(ctypes.byref(tmp))
        n_gains = tmp.value
        self.preamp_gains = []
        for i in range(n_gains):
            tmp = ctypes.c_float()
            self.dll.GetPreAmpGain(i, ctypes.byref(tmp))
            self.preamp_gains.append( round(tmp.value,1) )

    def set_preamp_gain(self, gain):
        """Sets the pre-amp gain. gain must be one of the values in self.preamp_gains"""
        try:
            index = self.preamp_gains.index(gain)
        except ValueError:
            raise ValueError("Pre-amp gain not in {}".format(self.preamp_gains))
        ret = self.dll.SetPreAmpGain(index)
        if ret != DRV_SUCCESS:
            raise Exception()

    def _get_vertical_shift_speeds(self):
        tmp = ctypes.c_int()
        self.dll.GetNumberVSSpeeds(ctypes.byref(tmp))
        n_gains = tmp.value
        self.vertical_shift_speeds = []
        for i in range(n_gains):
            tmp = ctypes.c_float()
            self.dll.GetVSSpeed(i, ctypes.byref(tmp))
            self.vertical_shift_speeds.append( round(tmp.value,1) )

    def set_vertical_shift_speed(self, vs_speed):
        """Sets the vertical shift speed in us. vs_speed must be one of the values in self.vertical_shift_speeds"""
        try:
            index = self.vertical_shift_speeds.index(vs_speed)
        except ValueError:
            raise ValueError("Vertical shift speed not in {}".format(self.vertical_shift_speeds))
        ret = self.dll.SetVSSpeed(index)
        if ret != DRV_SUCCESS:
            raise Exception()

    def set_vertical_clock_voltage(self, boost):
        """Set the vertical shift clock voltage boost.
        The valid range is integers between 0 (normal) and 4 (max boost)"""
        ret = self.dll.SetVSAmplitude(int(boost))
        if ret != DRV_SUCCESS:
            raise Exception()

    def _get_horizontal_shift_speeds(self):
        # Get the number of ADC channels
        tmp = ctypes.c_int()
        self.dll.GetNumberADChannels(ctypes.byref(tmp))
        n_adcs = tmp.value

        # Get the bit depths of the ADCs
        bit_depth = []
        for i in range(n_adcs):
            self.dll.GetBitDepth(i, ctypes.byref(tmp))
            bit_depth.append(tmp.value)

        em_gain = [True, False] # Maps gain_type to EM gain enabled

        # Populate the list of horizontal shift parameters
        self.horizontal_shift_parameters = []
        self._horiz_index = []
        self._adc_index = []
        for adc in range(n_adcs):
            for gain_type in range(2):
                self.dll.GetNumberHSSpeeds(adc, gain_type, ctypes.byref(tmp))
                n_speeds = tmp.value
                for i in range(n_speeds):
                    val = ctypes.c_float()
                    self.dll.GetHSSpeed(adc, gain_type, i, ctypes.byref(val))
                    self.horizontal_shift_parameters.append( (val.value, em_gain[gain_type], bit_depth[adc]) )
                    self._horiz_index.append(i)
                    self._adc_index.append(adc)

    def set_horizontal_shift_parameters(self, horizontal_shift_speed, em_gain=True, adc_bit_depth=14):
        """Set the horizontal pixel shift parameters. These are the shift speed (MHz), the ADC resolution, and the output
        amplifier used (EM gain or conventional).
        The tuple (horizontal_shift_speed, em_gain, adc_bit_depth) must be in self.horizontal_shift_parameters (i.e. it 
        must be a valid combination of parameters"""
        try:
            param_ind = self.horizontal_shift_parameters.index( (horizontal_shift_speed, em_gain, adc_bit_depth) )
        except ValueError:
            raise ValueError("Invalid combination of horizontal shift parameters")
        gain_type = int(not em_gain)

        ret = self.dll.SetADChannel(self._adc_index[param_ind])
        if ret != DRV_SUCCESS:
            raise Exception()

        ret = self.dll.SetHSSpeed(gain_type, self._horiz_index[param_ind])
        if ret != DRV_SUCCESS:
            raise Exception()

        ret = self.dll.SetOutputAmplifier(gain_type)
        if ret != DRV_SUCCESS:
            raise Exception()

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
        elif ret != DRV_SUCCESS or ret != DRV_TEMPERATURE_NOT_REACHED:
            raise Exception()
        return T.value

    def set_image_region(self, hStart, hEnd, vStart, vEnd, hBin=1, vBin=1):
        """Set the CCD region to read out and the horizontal and vertical binning.
        The region is 1 indexed and inclusive, so the valid ranges for hStart is 1..self.ccdWidth etc."""
        self.roiWidth = int((1+hEnd-hStart) / hBin)
        self.roiHeight = int((1+vEnd-vStart) / vBin)

        self.dll.SetReadMode(READMODE_IMAGE)
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

        # Record the size of the image circular buffer for the acquisition parameters we are using
        tmp = cbytes.c_long()
        self.dll.GetSizeOfCircularBuffer(cbytes.byref(tmp))
        self._circ_buffer_size = tmp.value

        if single:
            mode = ACQUISITION_SINGLE
        else:
            mode = ACQUISITION_RUN_TILL_ABORT

        ret = self.dll.SetAcquisitionMode(mode)
        if ret != DRV_SUCCESS:
            raise Exception()
        ret = self.dll.StartAcquisition()
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

    def get_all_images(self):
        """Returns all of the images in the buffer as an array of numpy arrays, or None if no new images"""
        first = ctypes.c_long()
        last = ctypes.c_long()
        ret = self.dll.GetNumberNewImages(ctypes.byref(first), ctypes.byref(last))
        if ret == DRV_NO_NEW_DATA:
            return None
        elif ret != DRV_SUCCESS:
            raise Exception()

        n_images = (last.value - first.value) % self._circ_buffer_size

        im_size = self.roiWidth*self.roiHeight
        buf = (ctypes.c_int * im_size * n_images)()
        valid_first = ctypes.c_long()
        valid_end = ctypes.c_long()
        ret = self.dll.GetImages(first, last, buf, ctypes.c_ulong(im_size * n_images),
            ctypes.byref(valid_first), ctypes.byref(valid_last))
        if ret != DRV_SUCCESS:
            raise Exception()

        assert(first.value == valid_first.value)
        assert(last.value == valid_last.value)

        im_array = []
        for i in n_images:
            im = np.frombuffer(buf[(im_size*i):im_size], dtype=np.int32)
            im = im.reshape(self.roiWidth, self.roiHeight)
            im = np.transpose(im)
            im_array.append(im)
        return im_array