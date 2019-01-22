import ctypes
import time
import platform
import os
import numpy as np
import collections
import threading

from ctypes import c_int, c_float, c_long, c_ulong, POINTER

# Constants for SetTrigger()
TRIGGER_INTERNAL = 0
TRIGGER_EXTERNAL = 1

# Constants for SetReadMode()
READMODE_SINGLETRACK = 3
READMODE_IMAGE = 4

# Constants for SetAcquitisionMode()
ACQUISITION_SINGLE = 2
ACQUISITION_RUN_TILL_ABORT = 5


class LibInstance:
    # Andor SDK return codes
    return_codes = {
        20002: "DRV_SUCCESS",
        20024: "DRV_NO_NEW_DATA",
        20034: "DRV_TEMPERATURE_OFF",
        20037: "DRV_TEMPERATURE_NOT_REACHED",
        20066: "DRV_P1INVALID",
        20067: "DRV_P2INVALID",
        20068: "DRV_P3INVALID",
        20069: "DRV_P4INVALID",
        20072: "DRV_ACQUIRING",
        20073: "DRV_IDLE",
        20075: "DRV_NOT_INITIALIZED"
    }

    def __init__(self):
        if platform.system() == "Windows":
            os.environ['PATH'] = "C:\Program Files\Andor SOLIS\Drivers" + ';' \
                                    + os.environ['PATH']
            if platform.architecture()[0] == "32bit":
                dll = ctypes.WinDLL("atmcd32d.dll")
            else:
                dll = ctypes.WinDLL("atmcd64d.dll")
            path = None
        elif platform.system() == "Linux":
            dll = ctypes.cdll.LoadLibrary("/usr/local/lib/libandor.so")
            path = "/usr/local/etc/andor".encode()
        else:
            raise Exception("Unsupported operating system")
        self.dll = dll

        def err_check(retval, ok_codes=[]):
            msg = self.return_codes.get(retval, "UNKNOWN")
            if msg not in ok_codes+["DRV_SUCCESS"]:
                raise Exception("Call failed with error '{}' ({})"
                    .format(msg, retval))

        def wrapper(f, argtypes=None, ok_codes=[]):
            f.restype = lambda x: err_check(x, ok_codes)
            if argtypes is not None:
                f.argtypes = argtypes
            return f

        def initialize():
            f = wrapper(dll.Initialize)
            if path:
                f(path)
            else:
                f()
        self.initialize = initialize

        def get_temperature():
            t = c_int()
            ret = dll.GetTemperature(ctypes.byref(t))
            if self.return_codes.get(ret, "UNKNOWN") == "DRV_TEMPERATURE_OFF":
                return None
            err_check(ret, ["DRV_SUCCESS", "DRV_TEMPERATURE_NOT_REACHED"])
            return t.value
        self.get_temperature = get_temperature

        def get_number_new_images(first, last):
            ret = dll.GetNumberNewImages(ctypes.byref(first),
                                              ctypes.byref(last))
            err_check(ret, ["DRV_NO_NEW_DATA"])
            if self.return_codes[ret] == "DRV_NO_NEW_DATA":
                return 0
            return (1+last.value - first.value) % (2**64)
        self.get_number_new_images = get_number_new_images

        self.abort_acquisition = wrapper(dll.AbortAcquisition, ok_codes=["DRV_IDLE"])
        self.set_cooler_mode = wrapper(dll.SetCoolerMode, [c_int])
        self.set_em_advanced = wrapper(dll.SetEMAdvanced, [c_int])
        self.set_em_gain_mode = wrapper(dll.SetEMGainMode, [c_int])
        self.get_detector = wrapper(dll.GetDetector, [POINTER(c_int), POINTER(c_int)])
        self.set_kinetic_cycle_time = wrapper(dll.SetKineticCycleTime, [c_int])
        self.get_number_preamp_gains = wrapper(dll.GetNumberPreAmpGains, [POINTER(c_int)])
        self.get_preamp_gain = wrapper(dll.GetPreAmpGain, [c_int, POINTER(c_float)])
        self.set_preamp_gain = wrapper(dll.SetPreAmpGain, [c_int])
        self.get_number_vs_speeds = wrapper(dll.GetNumberVSSpeeds, [POINTER(c_int)])
        self.get_vs_speed = wrapper(dll.GetVSSpeed, [c_int, POINTER(c_float)])
        self.set_vs_speed = wrapper(dll.SetVSSpeed, [c_int])
        self.set_vs_amplitude = wrapper(dll.SetVSAmplitude, [c_int])
        self.get_number_adc_channels = wrapper(dll.GetNumberADChannels, [POINTER(c_int)])
        self.get_bit_depth = wrapper(dll.GetBitDepth, [c_int, POINTER(c_int)])
        self.get_number_hs_speeds = wrapper(dll.GetNumberHSSpeeds,
            [c_int, c_int, POINTER(c_int)])
        self.get_hs_speed = wrapper(dll.GetHSSpeed,
            [c_int, c_int, c_int, POINTER(c_float)])
        self.set_adc_channel = wrapper(dll.SetADChannel, [c_int])
        self.set_hs_speed = wrapper(dll.SetHSSpeed, [c_int, c_int])
        self.set_output_amplifier = wrapper(dll.SetOutputAmplifier, [c_int])
        self.set_trigger_mode = wrapper(dll.SetTriggerMode, [c_int])
        self.cooler_on = wrapper(dll.CoolerON)
        self.set_temperature = wrapper(dll.SetTemperature, [c_int])
        self.set_read_mode = wrapper(dll.SetReadMode, [c_int])
        self.set_image = wrapper(dll.SetImage, [c_int]*6)
        self.set_em_gain = wrapper(dll.SetEMCCDGain, [c_int])
        self.set_exposure_time = wrapper(dll.SetExposureTime, [c_float])
        self.set_shutter = wrapper(dll.SetShutter, [c_int])
        self.get_acquisition_timings = wrapper(dll.GetAcquisitionTimings,
            [POINTER(c_float)]*3)
        self.set_acquisition_mode = wrapper(dll.SetAcquisitionMode, [c_int])
        self.start_acquisition = wrapper(dll.StartAcquisition)
        self.wait_for_acquisition = wrapper(dll.WaitForAcquisition)
        self.get_images = wrapper(dll.GetImages,
            [c_long, c_long, POINTER(c_int), c_ulong, POINTER(c_long), POINTER(c_long)])
        self.shutdown = wrapper(dll.ShutDown)
        self.get_available_cameras = wrapper(dll.GetAvailableCameras, [POINTER(c_int)])
        self.get_camera_handle = wrapper(dll.GetCameraHandle, [c_int, POINTER(c_int)])
        self.get_camera_serial = wrapper(dll.GetCameraSerialNumber, [POINTER(c_int)])
        self.set_current_camera = wrapper(dll.SetCurrentCamera, [c_int])


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

        self.lib = LibInstance()
        self.lib.initialize()

        self.lib.set_cooler_mode(not self.leave_camera_warm)

        # We are responsible adults - we can handle the full EM gain!
        self.lib.set_em_advanced(1)
        
        # Set 'real EM gain' mode
        self.lib.set_em_gain_mode(3)

        # Find the available gains, shift speeds, etc
        self._get_preamp_gains()
        self._get_vertical_shift_speeds()
        self._get_horizontal_shift_speeds()

        # Sensible non-EM parameters
        self.set_horizontal_shift_parameters(
            3, em_gain=False, adc_bit_depth=16)

        horiz = c_int()
        vert = c_int()
        self.lib.get_detector(ctypes.byref(horiz), ctypes.byref(vert))
        self.ccdWidth = horiz.value
        self.ccdHeight = vert.value

        # Set the default ROI to the full sensor
        self.set_image_region(0, self.ccdWidth-1, 0, self.ccdHeight-1, \
                                 hBin=1, vBin=1)

        # Sensible defaults
        self.set_shutter_open(True)
        self.set_trigger_mode(TRIGGER_INTERNAL)
        self.lib.set_kinetic_cycle_time(0)

        self.frame_buffer = collections.deque([], framebuffer_len)

        self._frame_call_list = []

        # Start image acquisition thread
        t = threading.Thread(target=self._acquisition_thread, daemon=True)
        t.start()

    def __del__(self):
        self.close()

    def close(self):
        """Leave the camera in a safe state and shutdown the driver"""
        if self.lib is None:
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
        self.lib.shutdown()
        self.lib = None

    def _get_preamp_gains(self):
        tmp = c_int()
        self.lib.get_number_preamp_gains(ctypes.byref(tmp))
        n_gains = tmp.value
        self.preamp_gains = []
        for i in range(n_gains):
            tmp = c_float()
            self.lib.get_preamp_gain(i, ctypes.byref(tmp))
            self.preamp_gains.append( round(tmp.value,1) )

    def set_preamp_gain(self, gain):
        """Sets the pre-amp gain. gain must be one of the values in 
        self.preamp_gains"""
        try:
            index = self.preamp_gains.index(gain)
        except ValueError:
            raise ValueError("Pre-amp gain not in {}".format(self.preamp_gains))
        self.lib.set_preamp_gain(index)

    def _get_vertical_shift_speeds(self):
        tmp = c_int()
        self.lib.get_number_vs_speeds(ctypes.byref(tmp))
        n_gains = tmp.value
        self.vertical_shift_speeds = []
        for i in range(n_gains):
            tmp = c_float()
            self.lib.get_vs_speed(i, ctypes.byref(tmp))
            self.vertical_shift_speeds.append( round(tmp.value,1) )

    def set_vertical_shift_speed(self, vs_speed):
        """Sets the vertical shift speed in us. vs_speed must be one of the 
        values in self.vertical_shift_speeds"""
        try:
            index = self.vertical_shift_speeds.index(vs_speed)
        except ValueError:
            raise ValueError("Vertical shift speed not in {}".format(
                                            self.vertical_shift_speeds))
        self.lib.set_vs_speed(index)

    def set_vertical_clock_voltage(self, boost):
        """Set the vertical shift clock voltage boost.
        The valid range is integers between 0 (normal) and 4 (max boost)"""
        self.lib.set_vs_amplitude(boost)

    def _get_horizontal_shift_speeds(self):
        # Get the number of ADC channels
        tmp = c_int()
        self.lib.get_number_adc_channels(ctypes.byref(tmp))
        n_adcs = tmp.value

        # Get the bit depths of the ADCs
        bit_depth = []
        for i in range(n_adcs):
            self.lib.get_bit_depth(i, ctypes.byref(tmp))
            bit_depth.append(tmp.value)

        em_gain = [True, False] # Maps gain_type to EM gain enabled

        # Populate the list of horizontal shift parameters
        self.horizontal_shift_parameters = []
        self._horiz_index = []
        self._adc_index = []
        for adc in range(n_adcs):
            for gain_type in range(2):
                self.lib.get_number_hs_speeds(adc, gain_type, ctypes.byref(tmp))
                n_speeds = tmp.value
                for i in range(n_speeds):
                    val = c_float()
                    self.lib.get_hs_speed(adc, gain_type, i, ctypes.byref(val))
                    self.horizontal_shift_parameters.append( 
                            (val.value, em_gain[gain_type], bit_depth[adc]) )
                    self._horiz_index.append(i)
                    self._adc_index.append(adc)

    def get_horizontal_shift_parameters(self):
        return self.horizontal_shift_parameters

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
        gain_type = not em_gain

        self.lib.set_adc_channel(self._adc_index[param_ind])
        self.lib.set_hs_speed(gain_type, self._horiz_index[param_ind])
        self.lib.set_output_amplifier(gain_type)

        self._em_gain_enabled = em_gain

    def set_trigger_mode(self, trig_mode):
        """Set the trigger mode between internal and external"""
        self.lib.set_trigger_mode(trig_mode)

    def set_temperature(self, temp):
        """Set the temperature setpoint to temp deg C"""
        self.lib.cooler_on()
        self.lib.set_temperature(temp)

    def get_temperature(self):
        """Returns the current camera temperature in deg C, or None if cooler
        is off"""
        return self.lib.get_temperature()

    def set_image_region(self, hStart, hEnd, vStart, vEnd, hBin=1, vBin=1):
        """Set the CCD region to read out and the horizontal and vertical
        binning.
        The region is 0 indexed and inclusive, so the valid ranges for hStart
        is 0..self.ccdWidth-1 etc."""
        self.roiWidth = int((1+hEnd-hStart) / hBin)
        self.roiHeight = int((1+vEnd-vStart) / vBin)

        self.lib.set_read_mode(READMODE_IMAGE)
        self.lib.set_image(int(hBin), int(vBin),  1+int(hStart),
                                                  1+int(hEnd),
                                                  1+int(vStart),
                                                  1+int(vEnd))


    def set_em_gain(self, gain):
        """Set the EM gain multiplication factor"""
        self.lib.SetEMCCDGain(gain)

    def set_exposure_time(self, time):
        """Set the CCD exposure time in seconds"""
        self.lib.set_exposure_time(time)

    def set_shutter_open(self, shutter_open):
        """Open / close the camera shutter"""
        self.lib.set_shutter(1, 1 if shutter_open else 2, 0, 0)

    def get_acquisition_timings(self):
        """Returns the actual timings the camera will use, after quantisation
        and padding as needed by the camera hardware.
        The timings are returned as a tuple (exposureTime, minCycleTime,
        minKineticTime)"""
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        ret = self.lib.get_acquisition_timings( ctypes.byref(exposure),
                                              ctypes.byref(accumulate),
                                              ctypes.byref(kinetic))
        return (exposure.value, accumulate.value, kinetic.value)

    def start_acquisition(self, single=False):
        """Start a single or repeated acquisition. If single=False the
        acquisition is repeated as fast as possible, (or on every trigger, if
        in 'external trigger' mode) until stop_acquisition() is called."""

        if single:
            mode = ACQUISITION_SINGLE
        else:
            mode = ACQUISITION_RUN_TILL_ABORT

        self.lib.set_acquisition_mode(mode)
        self.lib.start_acquisition()


    def stop_acquisition(self):
        """Stop a repeated acquisition"""
        self.lib.abort_acquisition()

    def wait_for_acquisition(self):
        """Wait for a new image to become available"""
        self.lib.wait_for_acquisition()


    def _get_all_images(self):
        """Returns all of the images in the camera buffer as an array of numpy
        arrays, or None if no new images"""
        first = c_long()
        last = c_long()
        n_images = self.lib.get_number_new_images(first, last)
        if n_images == 0:
            return None

        im_size = self.roiWidth*self.roiHeight
        buf = (c_int * im_size * n_images)()
        valid_first = c_long()
        valid_last = c_long()
        self.lib.get_images(first, last,
                            ctypes.cast(buf, POINTER(c_int)),
                            c_ulong(im_size * n_images),
                            ctypes.byref(valid_first), ctypes.byref(valid_last))


        assert(first.value == valid_first.value)
        assert(last.value == valid_last.value)

        im_array = []
        raw = np.frombuffer(buf, dtype=np.int32)
        for i in range(n_images):
            im = raw[(im_size*i):(im_size*(i+1))]
            im = im.reshape(self.roiHeight, self.roiWidth)
            im = np.transpose(im)
            if not self._em_gain_enabled:
                # Ensure image orientation is the same as for the EM amplifier
                im = np.flipud(im)
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
            # The GIL is released in the ctypes library call, so we get true
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

    def flush_images(self):
        """Delete all images from the buffer"""
        while len(self.frame_buffer) > 0:
            self.frame_buffer.popleft()

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
