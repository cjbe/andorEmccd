import ctypes
import time
import platform
import os
import numpy as np
import collections
import threading

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
DRV_IDLE = 20073
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
    Note that almost all camera parameters can only be changed when the camera
    is not acquiring"""
    dll = None

    def __init__(self, leave_camera_warm=True, framebuffer_len=100):
        """Initialise the camera interface.

        leave_camera_warm: turn off the cooler when disconnecting from the
        camera.

        framebuffer_len: maximum number of stored frames before oldest are discarded
        """
        self.leave_camera_warm = leave_camera_warm

        if platform.system() == "Windows":
            os.environ['PATH'] = "C:\Program Files\Andor SOLIS\Drivers" + ';' \
                                    + os.environ['PATH']
            if platform.architecture()[0] == "32bit":
                self.dll = ctypes.WinDLL("atmcd32d.dll")
            else:
                self.dll = ctypes.WinDLL("atmcd64d.dll")
            ret = self.dll.Initialize()
        elif platform.system() == "Linux":
            self.dll = ctypes.cdll.LoadLibrary("/usr/local/lib/libandor.so")
            ret = self.dll.Initialize("/usr/local/etc/andor".encode())
        else:
            raise Exception("Unsupported operating system")

        if ret != DRV_SUCCESS:
            self.dll = None
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
        self.set_image_region(0, self.ccdWidth-1, 0, self.ccdHeight-1, \
                                 hBin=1, vBin=1)

        # Sensible defaults
        self.set_shutter_open(True)
        self.set_trigger_mode(TRIGGER_INTERNAL)
        self.dll.SetKineticCycleTime(0)

        self.frame_buffer = collections.deque([], framebuffer_len)

        self._frame_call_list = []

        # Start image acquisition thread
        t = threading.Thread(target=self._acquisition_thread, daemon=True)
        t.start()

    def __del__(self):
        if self.dll is not None:
            self.close()

    def close(self):
        """Leave the camera in a safe state and shutdown the driver"""
        if self.dll is None:
            return
        if self.leave_camera_warm:
            self.stop_acquisition()
            self.set_temperature(10)
            print("Waiting for camera to warm up ...")
            while self.get_temperature() < -20:
                # Wait for camera to warm up into the safe temperature range 
                # of >-20
                time.sleep(1)
            print("Camera now at T={}".format(self.get_temperature()))
        self.dll.ShutDown()
        self.dll = None

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
        """Sets the pre-amp gain. gain must be one of the values in 
        self.preamp_gains"""
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
        """Sets the vertical shift speed in us. vs_speed must be one of the 
        values in self.vertical_shift_speeds"""
        try:
            index = self.vertical_shift_speeds.index(vs_speed)
        except ValueError:
            raise ValueError("Vertical shift speed not in {}".format(
                                            self.vertical_shift_speeds))
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
                    self.horizontal_shift_parameters.append( 
                            (val.value, em_gain[gain_type], bit_depth[adc]) )
                    self._horiz_index.append(i)
                    self._adc_index.append(adc)

    def set_horizontal_shift_parameters(self, horizontal_shift_speed,
                                              em_gain=True,
                                              adc_bit_depth=14):
        """Set the horizontal pixel shift parameters. These are the shift speed
        (MHz), the ADC resolution, and the output amplifier used (EM gain or
        conventional).
        The tuple (horizontal_shift_speed, em_gain, adc_bit_depth) must be in
        self.horizontal_shift_parameters (i.e. it must be a valid combination
        of parameters"""
        try:
            param_ind = self.horizontal_shift_parameters.index( 
                            (horizontal_shift_speed, em_gain, adc_bit_depth) )
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
        """Returns the current camera temperature in deg C, or None if cooler
        is off"""
        T = ctypes.c_int()
        ret = self.dll.GetTemperature(ctypes.byref(T))
        if ret == DRV_TEMPERATURE_OFF:
            return None
        elif ret != DRV_SUCCESS and ret != DRV_TEMPERATURE_NOT_REACHED:
            raise Exception()
        return T.value

    def set_image_region(self, hStart, hEnd, vStart, vEnd, hBin=1, vBin=1):
        """Set the CCD region to read out and the horizontal and vertical
        binning.
        The region is 0 indexed and inclusive, so the valid ranges for hStart
        is 0..self.ccdWidth-1 etc."""
        self.roiWidth = int((1+hEnd-hStart) / hBin)
        self.roiHeight = int((1+vEnd-vStart) / vBin)

        self.dll.SetReadMode(READMODE_IMAGE)
        ret = self.dll.SetImage(int(hBin), int(vBin), 1+int(hStart),
                                                      1+int(hEnd),
                                                      1+int(vStart),
                                                      1+int(vEnd))
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

    def get_acquisition_timings(self):
        """Returns the actual timings the camera will use, after quantisation
        and padding as needed by the camera hardware.
        The timings are returned as a tuple (exposureTime, minCycleTime,
        minKineticTime)"""
        exposure = ctypes.c_float()
        accumulate = ctypes.c_float()
        kinetic = ctypes.c_float()
        ret = self.dll.GetAcquisitionTimings( ctypes.byref(exposure),
                                              ctypes.byref(accumulate),
                                              ctypes.byref(kinetic))
        if ret != DRV_SUCCESS:
            raise Exception()

        return (exposure.value, accumulate.value, kinetic.value)

    def start_acquisition(self, single=False):
        """Start a single or repeated acquisition. If single=False the
        acquisition is repeated as fast as possible, (or on every trigger, if
        in 'external trigger' mode) until stop_acquisition() is called."""

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
        if ret != DRV_SUCCESS and ret != DRV_IDLE:
            raise Exception()

    def wait_for_acquisition(self):
        """Wait for a new image to become available"""
        ret = self.dll.WaitForAcquisition()
        if ret != DRV_SUCCESS:
            raise Exception()

    def _get_all_images(self):
        """Returns all of the images in the camera buffer as an array of numpy
        arrays, or None if no new images"""
        first = ctypes.c_long()
        last = ctypes.c_long()
        ret = self.dll.GetNumberNewImages(ctypes.byref(first),
                                          ctypes.byref(last))
        if ret == DRV_NO_NEW_DATA:
            return None
        elif ret != DRV_SUCCESS:
            raise Exception()

        n_images = (1+last.value - first.value) % (2**64)

        im_size = self.roiWidth*self.roiHeight
        buf = (ctypes.c_int * im_size * n_images)()
        valid_first = ctypes.c_long()
        valid_last = ctypes.c_long()
        ret = self.dll.GetImages(first, last, buf,
                            ctypes.c_ulong(im_size * n_images),
                            ctypes.byref(valid_first), ctypes.byref(valid_last))
        if ret != DRV_SUCCESS:
            raise Exception()

        assert(first.value == valid_first.value)
        assert(last.value == valid_last.value)

        im_array = []
        raw = np.frombuffer(buf, dtype=np.int32)
        for i in range(n_images):
            im = raw[(im_size*i):(im_size*(i+1))]
            im = im.reshape(self.roiHeight, self.roiWidth)
            im = np.transpose(im)
            im_array.append(im.copy(order="C"))
        return im_array

    def register_callback(self, f):
        """Register a function to be called from the acquisition thread for each
        new image"""
        self._frame_call_list.append(f)

    def deregister_callback(self, f):
        if f in self._frame_call_list:
            self._frame_call_list.remove(f)

    def _acquisition_thread(self):
        while True:
            # The GIL is released in the cytes library call, so we get true
            # multithreading until wait_for_acquisition() returns
            self.wait_for_acquisition()
            ims = self._get_all_images()
            if ims is None:
                continue
            for im in ims:
                self.frame_buffer.append(im)
                for f in self._frame_call_list:
                    f(im)

    def get_image(self):
        """Returns the oldest image in the buffer as a numpy array, or None if
        no new images"""
        if len(self.frame_buffer) == 0:
            return None
        return self.frame_buffer.popleft()

    def get_all_images(self):
        """Returns all of the images in the buffer as an array of numpy arrays,
        or None if no new images"""
        if len(self.frame_buffer):
            ims = []
            while len(self.frame_buffer) > 0:
                ims.append(self.frame_buffer.popleft())
        else:
            ims = None
        return ims
