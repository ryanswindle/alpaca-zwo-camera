from ctypes import (
    POINTER,
    c_int,
    c_long,
    c_ubyte,
)
from datetime import datetime, timezone
from enum import IntEnum
from threading import Event, Lock, Thread
from typing import Dict, List, Optional

from astropy.time import Time
import numpy as np
import time

from libasicamera2 import (
    ASI_BOOL,
    ASI_CAMERA_INFO,
    ASI_CONTROL_CAPS,
    ASI_CONTROL_TYPE,
    ASI_ERROR_CODE,
    ASI_EXPOSURE_STATUS,
    ASI_IMG_TYPE,
    ASIError,
    asi_call,
    load_asi_library,
)
from config import DeviceConfig
from log import get_logger


logger = get_logger()


class CameraState(IntEnum):
    IDLE = 0
    WAITING = 1
    EXPOSING = 2
    READING = 3
    DOWNLOADING = 4
    ERROR = 5


class SensorType(IntEnum):
    MONOCHROME = 0
    COLOR = 1
    RGGB = 2
    CMYG = 3
    CMYG2 = 4
    LRGB = 5


class CameraDevice:
    """Low-level driver for ZWO cameras (libASICamera2)."""

    def __init__(self, device_config: DeviceConfig, library_path: str):
        self._lock = Lock()
        self._config = device_config
        self._library_path = library_path

        self._libasicamera2 = None
        self._camera_info: Optional[ASI_CAMERA_INFO] = None
        self._camera_id: int = -1
        self._control_caps: Dict[int, ASI_CONTROL_CAPS] = {}

        self._connected = False
        self._connecting = False
        self._connect_thread: Optional[Thread] = None
        self._disconnect_thread: Optional[Thread] = None

        self._camera_state = CameraState.IDLE
        self._image_ready = False
        self._exposure_complete = Event()

        self._last_exposure_duration: Optional[float] = None
        self._last_exposure_start_time: Optional[str] = None
        self._exposure_thread: Optional[Thread] = None

        self._image_buffer: Optional[np.ndarray] = None

    #######################################
    # ASCOM Methods Common To All Devices #
    #######################################
    def connect(self) -> None:
        if self._connected or self._connecting:
            return
        self._connecting = True
        self._connect_thread = Thread(target=self._connect_worker, daemon=True)
        self._connect_thread.start()

    def _connect_worker(self) -> None:
        """Load the library, open the camera, query and set parameters."""
        try:
            # Load the library
            if self._libasicamera2 is None:
                self._libasicamera2 = load_asi_library(self._library_path)
                if self._libasicamera2 is None:
                    raise RuntimeError("Failed to load ASICamera2 library")

            # SDK version
            sdk_ver = self._libasicamera2.ASIGetSDKVersion()
            logger.debug(f"ASICamera2 SDK version: {sdk_ver.decode() if isinstance(sdk_ver, bytes) else sdk_ver}")

            # Discover cameras
            num_cameras = self._libasicamera2.ASIGetNumOfConnectedCameras()
            if num_cameras == 0:
                raise RuntimeError(
                    "No ASI cameras found. "
                    "Check connection and ensure no other application has the camera open."
                )
            logger.info(f"Found {num_cameras} ASI camera(s)")

            # Find our camera by serial number or use first available
            camera_index = 0
            for i in range(num_cameras):
                info = ASI_CAMERA_INFO()
                asi_call(
                    self._libasicamera2.ASIGetCameraProperty,
                    POINTER(ASI_CAMERA_INFO)(info),
                    c_int(i),
                    operation="GetCameraProperty",
                )
                cam_name = info.Name.decode("utf-8", errors="replace").strip()
                logger.debug(f"Camera {i}: {cam_name}")
                if self._config.serial_number:
                    if str(info.CameraID) == self._config.serial_number:
                        camera_index = i
                        break
                elif i == self._config.device_number:
                    camera_index = i
                    break

            # Get camera info
            self._camera_info = ASI_CAMERA_INFO()
            asi_call(
                self._libasicamera2.ASIGetCameraProperty,
                POINTER(ASI_CAMERA_INFO)(self._camera_info),
                c_int(camera_index),
                operation="GetCameraProperty",
            )
            self._camera_id = self._camera_info.CameraID

            # Open and init
            asi_call(self._libasicamera2.ASIOpenCamera, c_int(self._camera_id), operation="OpenCamera")
            asi_call(self._libasicamera2.ASIInitCamera, c_int(self._camera_id), operation="InitCamera")

            # Now query camera properties from the SDK
            self._query_camera_properties()

            # Set remaining default parameters (temperature, gain, offset, binning, etc.)
            self._set_default_parameters()

            self._connected = True
            self._camera_state = CameraState.IDLE
            self._image_ready = False
            logger.info(f"Connected to camera {self._config.entity}")

        except Exception as e:
            logger.error(f"Connection failed for {self._config.entity}: {e}")
            self._connected = False
            self._camera_state = CameraState.ERROR
            raise
        finally:
            self._connecting = False

    def _query_camera_properties(self) -> None:
        logger.debug(f"querying camera properties for {self._config.entity}")
        info = self._camera_info

        # Frame size
        self._camera_x_size = int(info.MaxWidth)
        self._camera_y_size = int(info.MaxHeight)
        logger.debug(f"MaxWidth={self._camera_x_size}, MaxHeight={self._camera_y_size}")

        # Pixel size (um)
        self._pixel_size_x = info.PixelSize
        self._pixel_size_y = info.PixelSize
        logger.debug(f"PixelSize={self._pixel_size_x} um")

        # Sensor name
        self._sensor_name = info.Name.decode("utf-8", errors="replace").strip()

        # Color / Bayer
        self._is_color = bool(info.IsColorCam)
        self._bayer_pattern = int(info.BayerPattern)

        # Mechanical shutter
        self._has_shutter = bool(info.MechanicalShutter)

        # ST4 port
        self._has_st4 = bool(info.ST4Port)

        # Cooler
        self._is_cooler_cam = bool(info.IsCoolerCam)

        # Bit depth
        self._adc_bit_depth = int(info.BitDepth)
        logger.debug(f"BitDepth={self._adc_bit_depth}")

        # Electrons per ADU
        self._elec_per_adu = float(info.ElecPerADU)
        logger.debug(f"ElecPerADU={self._elec_per_adu}")

        # Supported bins
        self._available_binnings = []
        for i in range(16):
            b = info.SupportedBins[i]
            if b == 0:
                break
            self._available_binnings.append(b)
        if not self._available_binnings:
            self._available_binnings = [1]
        self._max_bin_x = max(self._available_binnings)
        self._max_bin_y = max(self._available_binnings)
        logger.debug(f"SupportedBins={self._available_binnings}")

        # Supported image formats
        self._supported_formats = []
        for i in range(8):
            fmt = info.SupportedVideoFormat[i]
            if fmt == ASI_IMG_TYPE.END:
                break
            self._supported_formats.append(fmt)
        logger.debug(f"SupportedVideoFormats={self._supported_formats}")

        # Query all control capabilities
        num_controls = c_int()
        asi_call(
            self._libasicamera2.ASIGetNumOfControls,
            c_int(self._camera_id),
            POINTER(c_int)(num_controls),
            operation="GetNumOfControls",
        )
        logger.debug(f"number of controls: {num_controls.value}")

        self._control_caps = {}
        for i in range(num_controls.value):
            caps = ASI_CONTROL_CAPS()
            asi_call(
                self._libasicamera2.ASIGetControlCaps,
                c_int(self._camera_id),
                c_int(i),
                POINTER(ASI_CONTROL_CAPS)(caps),
                operation="GetControlCaps",
            )
            self._control_caps[caps.ControlType] = caps
            logger.debug(
                f"  control {i}: {caps.Name.decode()} type={caps.ControlType} "
                f"min={caps.MinValue} max={caps.MaxValue} default={caps.DefaultValue}"
            )

        # Extract exposure limits from control caps (SDK unit: microseconds)
        if ASI_CONTROL_TYPE.EXPOSURE in self._control_caps:
            exp_caps = self._control_caps[ASI_CONTROL_TYPE.EXPOSURE]
            self._exposure_min = exp_caps.MinValue / 1_000_000.0
            self._exposure_max = exp_caps.MaxValue / 1_000_000.0
            self._exposure_resolution = 0.000001  # 1 us
        else:
            self._exposure_min = 0.0
            self._exposure_max = 3600.0
            self._exposure_resolution = 0.000001

        # Gain limits
        if ASI_CONTROL_TYPE.GAIN in self._control_caps:
            gain_caps = self._control_caps[ASI_CONTROL_TYPE.GAIN]
            self._gain_min = int(gain_caps.MinValue)
            self._gain_max = int(gain_caps.MaxValue)
        else:
            self._gain_min = 0
            self._gain_max = 0

        # Offset limits
        if ASI_CONTROL_TYPE.BRIGHTNESS in self._control_caps:
            offset_caps = self._control_caps[ASI_CONTROL_TYPE.BRIGHTNESS]
            self._offset_min = int(offset_caps.MinValue)
            self._offset_max = int(offset_caps.MaxValue)
        else:
            self._offset_min = 0
            self._offset_max = 0

        # Build readout modes from config (gain presets) or single default
        if self._config.readout_modes:
            self._readout_modes = [mode.label for mode in self._config.readout_modes]
            self._readout_mode_gains = [mode.gain for mode in self._config.readout_modes]
        else:
            self._readout_modes = [f"Gain_{self._config.defaults.gain}"]
            self._readout_mode_gains = [self._config.defaults.gain]

    def _set_default_parameters(self) -> None:
        logger.debug(f"setting default parameters for {self._config.entity}")
        defaults = self._config.defaults

        # Temperature target (cooled cameras only)
        if self._is_cooler_cam:
            self._set_control(ASI_CONTROL_TYPE.TARGET_TEMP, int(defaults.temperature))
            self._set_control(ASI_CONTROL_TYPE.COOLER_ON, 1)

        # Gain
        self._set_control(ASI_CONTROL_TYPE.GAIN, defaults.gain)

        # Offset
        if ASI_CONTROL_TYPE.BRIGHTNESS in self._control_caps:
            self._set_control(ASI_CONTROL_TYPE.BRIGHTNESS, defaults.offset)

        # Readout mode index
        self._readout_mode = defaults.readout_mode

        # ROI: full frame, default binning, RAW16
        self._bin_x = self._bin_y = (
            defaults.binning if defaults.binning in self._available_binnings else 1
        )
        self._img_type = ASI_IMG_TYPE.RAW16 if ASI_IMG_TYPE.RAW16 in self._supported_formats else ASI_IMG_TYPE.RAW8

        width = self._camera_x_size // self._bin_x
        height = self._camera_y_size // self._bin_y
        # ASI SDK requires width%8==0, height%2==0
        width = (width // 8) * 8
        height = (height // 2) * 2

        asi_call(
            self._libasicamera2.ASISetROIFormat,
            c_int(self._camera_id),
            c_int(width),
            c_int(height),
            c_int(self._bin_x),
            c_int(self._img_type),
            operation="SetROIFormat",
        )
        asi_call(
            self._libasicamera2.ASISetStartPos,
            c_int(self._camera_id),
            c_int(0),
            c_int(0),
            operation="SetStartPos",
        )

        self._start_x = 0
        self._start_y = 0
        self._num_x = width
        self._num_y = height

        # USB bandwidth
        if ASI_CONTROL_TYPE.BANDWIDTHOVERLOAD in self._control_caps:
            self._set_control(ASI_CONTROL_TYPE.BANDWIDTHOVERLOAD, defaults.usb_bandwidth)

        # High speed mode off for long exposures
        if ASI_CONTROL_TYPE.HIGH_SPEED_MODE in self._control_caps:
            self._set_control(ASI_CONTROL_TYPE.HIGH_SPEED_MODE, 0)

    def _set_control(self, control_type: int, value: int, auto: bool = False) -> None:
        asi_call(
            self._libasicamera2.ASISetControlValue,
            c_int(self._camera_id),
            c_int(control_type),
            c_long(value),
            c_int(ASI_BOOL.TRUE if auto else ASI_BOOL.FALSE),
            operation=f"SetControlValue({control_type}={value})",
        )

    def _get_control(self, control_type: int) -> int:
        value = c_long()
        auto = c_int()
        asi_call(
            self._libasicamera2.ASIGetControlValue,
            c_int(self._camera_id),
            c_int(control_type),
            POINTER(c_long)(value),
            POINTER(c_int)(auto),
            operation=f"GetControlValue({control_type})",
        )
        return int(value.value)

    @property
    def connected(self) -> bool:
        return self._connected

    @connected.setter
    def connected(self, value: bool) -> None:
        if value and not self._connected:
            self.connect()
        elif not value and self._connected:
            self.disconnect()

    @property
    def connecting(self) -> bool:
        return self._connecting

    def disconnect(self) -> None:
        if not self._connected and not self._connecting:
            return
        self._disconnect_thread = Thread(target=self._disconnect_worker, daemon=True)
        self._disconnect_thread.start()

    def _disconnect_worker(self) -> None:
        try:
            if self._camera_state in (CameraState.EXPOSING, CameraState.READING):
                self.abort_exposure()
            if self._camera_id >= 0 and self._libasicamera2:
                self._libasicamera2.ASICloseCamera(c_int(self._camera_id))
            self._connected = False
            self._camera_state = CameraState.IDLE
            logger.info(f"Disconnected from camera {self._config.entity}")
        except Exception as e:
            logger.error(f"Disconnect error for {self._config.entity}: {e}")
        finally:
            self._connecting = False

    ######################
    # ICamera properties #
    ######################
    @property
    def bin_x(self) -> int:
        return self._bin_x

    @bin_x.setter
    def bin_x(self, value: int) -> None:
        self._set_binning(value)

    @property
    def bin_y(self) -> int:
        return self._bin_y

    @bin_y.setter
    def bin_y(self, value: int) -> None:
        self._set_binning(value)

    def _set_binning(self, value: int) -> None:
        if value not in self._available_binnings:
            raise ValueError(
                f"Bin {value} not in available binnings {self._available_binnings}"
            )
        self._bin_x = self._bin_y = value

        # Reset to full frame at new binning
        width = self._camera_x_size // value
        height = self._camera_y_size // value
        width = (width // 8) * 8
        height = (height // 2) * 2

        asi_call(
            self._libasicamera2.ASISetROIFormat,
            c_int(self._camera_id),
            c_int(width),
            c_int(height),
            c_int(value),
            c_int(self._img_type),
            operation="SetROIFormat",
        )
        asi_call(
            self._libasicamera2.ASISetStartPos,
            c_int(self._camera_id),
            c_int(0),
            c_int(0),
            operation="SetStartPos",
        )

        self._start_x = 0
        self._start_y = 0
        self._num_x = width
        self._num_y = height

    @property
    def camera_state(self) -> CameraState:
        return self._camera_state

    @property
    def camera_x_size(self) -> int:
        return self._camera_x_size

    @property
    def camera_y_size(self) -> int:
        return self._camera_y_size

    @property
    def can_abort_exposure(self) -> bool:
        return True

    @property
    def can_asymmetric_bin(self) -> bool:
        return False

    @property
    def can_fast_readout(self) -> bool:
        return False

    @property
    def can_get_cooler_power(self) -> bool:
        return self._is_cooler_cam

    @property
    def can_pulse_guide(self) -> bool:
        return self._has_st4

    @property
    def can_set_ccd_temperature(self) -> bool:
        return self._is_cooler_cam

    @property
    def can_stop_exposure(self) -> bool:
        return True

    @property
    def ccd_temperature(self) -> float:
        try:
            raw = self._get_control(ASI_CONTROL_TYPE.TEMPERATURE)
            return raw / 10.0  # SDK returns 10x actual
        except ASIError:
            logger.warning("Unable to read temperature")
            return 99.0

    @property
    def cooler_on(self) -> bool:
        if not self._is_cooler_cam:
            return False
        try:
            return bool(self._get_control(ASI_CONTROL_TYPE.COOLER_ON))
        except ASIError:
            return False

    @cooler_on.setter
    def cooler_on(self, value: bool) -> None:
        if self._is_cooler_cam:
            self._set_control(ASI_CONTROL_TYPE.COOLER_ON, 1 if value else 0)

    @property
    def cooler_power(self) -> float:
        if not self._is_cooler_cam:
            return 0.0
        try:
            return float(self._get_control(ASI_CONTROL_TYPE.COOLER_POWER_PERC))
        except ASIError:
            return 0.0

    @property
    def electrons_per_adu(self) -> float:
        return self._elec_per_adu

    @property
    def exposure_max(self) -> float:
        return self._exposure_max

    @property
    def exposure_min(self) -> float:
        return self._exposure_min

    @property
    def exposure_resolution(self) -> float:
        return self._exposure_resolution

    @property
    def gain(self) -> int:
        try:
            return self._get_control(ASI_CONTROL_TYPE.GAIN)
        except ASIError:
            return 0

    @gain.setter
    def gain(self, value: int) -> None:
        if value < self._gain_min or value > self._gain_max:
            raise ValueError(f"Gain {value} out of range [{self._gain_min}, {self._gain_max}]")
        self._set_control(ASI_CONTROL_TYPE.GAIN, value)

    @property
    def gain_max(self) -> int:
        return self._gain_max

    @property
    def gain_min(self) -> int:
        return self._gain_min

    @property
    def has_shutter(self) -> bool:
        return self._has_shutter

    @property
    def image_array(self) -> np.ndarray:
        if not self._image_ready:
            raise RuntimeError("No image ready")
        if self._image_buffer is None:
            raise RuntimeError("No image data available")

        self._camera_state = CameraState.DOWNLOADING

        width, height = self._num_x, self._num_y

        if self._img_type == ASI_IMG_TYPE.RAW16:
            img = np.frombuffer(self._image_buffer, dtype=np.uint16).reshape((height, width))
        elif self._img_type == ASI_IMG_TYPE.RAW8 or self._img_type == ASI_IMG_TYPE.Y8:
            img = np.frombuffer(self._image_buffer, dtype=np.uint8).reshape((height, width))
        else:
            # RGB24: 3 bytes per pixel
            img = np.frombuffer(self._image_buffer, dtype=np.uint8).reshape((height, width, 3))

        logger.debug(
            f"got data with {img.shape} dtype={img.dtype}"
        )

        self._camera_state = CameraState.IDLE
        self._image_ready = False

        return img.astype(np.int32)

    @property
    def image_ready(self) -> bool:
        return self._image_ready

    @property
    def last_exposure_duration(self) -> float:
        return self._last_exposure_duration

    @property
    def last_exposure_start_time(self) -> str:
        return self._last_exposure_start_time

    @property
    def max_adu(self) -> int:
        return int((1 << self._adc_bit_depth) - 1)

    @property
    def max_bin_x(self) -> int:
        return self._max_bin_x

    @property
    def max_bin_y(self) -> int:
        return self._max_bin_y

    @property
    def num_x(self) -> int:
        return self._num_x

    @num_x.setter
    def num_x(self, value: int) -> None:
        self._set_roi(num_x=value)

    @property
    def num_y(self) -> int:
        return self._num_y

    @num_y.setter
    def num_y(self, value: int) -> None:
        self._set_roi(num_y=value)

    @property
    def offset(self) -> int:
        try:
            return self._get_control(ASI_CONTROL_TYPE.BRIGHTNESS)
        except ASIError:
            return 0

    @offset.setter
    def offset(self, value: int) -> None:
        if value < self._offset_min or value > self._offset_max:
            raise ValueError(f"Offset {value} out of range [{self._offset_min}, {self._offset_max}]")
        self._set_control(ASI_CONTROL_TYPE.BRIGHTNESS, value)

    @property
    def offset_max(self) -> int:
        return self._offset_max

    @property
    def offset_min(self) -> int:
        return self._offset_min

    @property
    def pixel_size_x(self) -> float:
        return self._pixel_size_x

    @property
    def pixel_size_y(self) -> float:
        return self._pixel_size_y

    @property
    def readout_mode(self) -> int:
        return self._readout_mode

    @readout_mode.setter
    def readout_mode(self, value: int) -> None:
        if value < 0 or value >= len(self._readout_modes):
            raise ValueError(
                f"ReadoutMode {value} out of range [0, {len(self._readout_modes) - 1}]"
            )
        self._readout_mode = value
        gain = self._readout_mode_gains[value]
        self._set_control(ASI_CONTROL_TYPE.GAIN, gain)
        logger.info(f"Set readout mode to {self._readout_modes[value]} (gain={gain})")

    @property
    def readout_modes(self) -> List[str]:
        return self._readout_modes

    @property
    def sensor_name(self) -> str:
        return self._sensor_name

    @property
    def sensor_type(self) -> SensorType:
        if self._is_color:
            return SensorType.COLOR
        return SensorType.MONOCHROME

    @property
    def set_ccd_temperature(self) -> float:
        if not self._is_cooler_cam:
            return 99.0
        try:
            return float(self._get_control(ASI_CONTROL_TYPE.TARGET_TEMP))
        except ASIError:
            logger.warning("Unable to read temperature set point")
            return 99.0

    @set_ccd_temperature.setter
    def set_ccd_temperature(self, value: float) -> None:
        if self._is_cooler_cam:
            try:
                self._set_control(ASI_CONTROL_TYPE.TARGET_TEMP, int(value))
                logger.debug(f"set ccd temperature to {value}")
            except ASIError:
                logger.warning("Unable to set ccd temperature")

    @property
    def start_x(self) -> int:
        return self._start_x

    @start_x.setter
    def start_x(self, value: int) -> None:
        self._set_roi(start_x=value)

    @property
    def start_y(self) -> int:
        return self._start_y

    @start_y.setter
    def start_y(self, value: int) -> None:
        self._set_roi(start_y=value)

    @property
    def timestamp(self) -> str:
        return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    def _set_roi(
        self, start_x=None, num_x=None, start_y=None, num_y=None
    ) -> None:
        """
        Set ROI with proper validation.

        All start/num values are in binned pixels per ASCOM spec.
        The ASI SDK also works in binned pixels for ROI width/height
        and start position (after ASISetROIFormat sets the binning).
        """
        sx = start_x if start_x is not None else self._start_x
        sy = start_y if start_y is not None else self._start_y
        nx = num_x if num_x is not None else self._num_x
        ny = num_y if num_y is not None else self._num_y

        # Max binned dimensions
        max_binned_x = self._camera_x_size // self._bin_x
        max_binned_y = self._camera_y_size // self._bin_y

        # Validate and clamp start values
        if sx < 0:
            sx = 0
        if sy < 0:
            sy = 0
        if sx >= max_binned_x:
            sx = max_binned_x - 1
        if sy >= max_binned_y:
            sy = max_binned_y - 1

        # Validate and clamp num values
        max_nx = max_binned_x - sx
        max_ny = max_binned_y - sy
        if nx < 1:
            nx = 1
        if ny < 1:
            ny = 1
        if nx > max_nx:
            nx = max_nx
        if ny > max_ny:
            ny = max_ny

        # ASI SDK alignment: width%8==0, height%2==0
        nx = (nx // 8) * 8
        ny = (ny // 2) * 2
        if nx < 8:
            nx = 8
        if ny < 2:
            ny = 2

        # Set ROI format (width, height, bin, img_type)
        asi_call(
            self._libasicamera2.ASISetROIFormat,
            c_int(self._camera_id),
            c_int(nx),
            c_int(ny),
            c_int(self._bin_x),
            c_int(self._img_type),
            operation="SetROIFormat",
        )

        # Set start position
        asi_call(
            self._libasicamera2.ASISetStartPos,
            c_int(self._camera_id),
            c_int(sx),
            c_int(sy),
            operation="SetStartPos",
        )

        self._start_x = sx
        self._start_y = sy
        self._num_x = nx
        self._num_y = ny

    ###################
    # ICamera methods #
    ###################
    def start_exposure(self, duration: float, light: bool) -> None:
        if self._camera_state != CameraState.IDLE:
            raise RuntimeError("Camera is not idle")
        self._image_ready = False
        self._camera_state = CameraState.WAITING
        self._exposure_complete.clear()
        self._exposure_thread = Thread(
            target=self._exposure_worker, args=(duration, light), daemon=True
        )
        self._exposure_thread.start()

    def _exposure_worker(self, duration: float, light: bool) -> None:
        try:
            # Set exposure time (SDK unit: microseconds)
            exposure_us = int(duration * 1_000_000)
            self._set_control(ASI_CONTROL_TYPE.EXPOSURE, exposure_us)

            self._last_exposure_start_time = Time.now().isot
            self._last_exposure_duration = duration

            # Start snap-shot exposure
            # ASI_BOOL isDark: ASI_FALSE for light, ASI_TRUE for dark
            asi_call(
                self._libasicamera2.ASIStartExposure,
                c_int(self._camera_id),
                c_int(ASI_BOOL.FALSE if light else ASI_BOOL.TRUE),
                operation="StartExposure",
            )

            self._camera_state = CameraState.EXPOSING
            logger.debug(f"starting exposure ({duration}s)")

            # Poll exposure status
            status = c_int()
            timeout = duration + 60.0
            t0 = time.time()

            while True:
                asi_call(
                    self._libasicamera2.ASIGetExpStatus,
                    c_int(self._camera_id),
                    POINTER(c_int)(status),
                    operation="GetExpStatus",
                )

                if status.value == ASI_EXPOSURE_STATUS.SUCCESS:
                    break
                elif status.value == ASI_EXPOSURE_STATUS.FAILED:
                    raise RuntimeError("Exposure failed (ASI_EXP_FAILED)")
                elif status.value == ASI_EXPOSURE_STATUS.IDLE:
                    raise RuntimeError("Exposure unexpectedly idle")

                if (time.time() - t0) > timeout:
                    raise RuntimeError(f"Exposure timed out after {timeout}s")

                time.sleep(0.1)

            logger.debug("exposure complete, reading data")
            self._camera_state = CameraState.READING

            # Calculate buffer size
            width, height = self._num_x, self._num_y
            if self._img_type == ASI_IMG_TYPE.RAW16:
                buf_size = width * height * 2
            elif self._img_type == ASI_IMG_TYPE.RGB24:
                buf_size = width * height * 3
            else:
                buf_size = width * height

            # Read image data
            buf = (c_ubyte * buf_size)()
            asi_call(
                self._libasicamera2.ASIGetDataAfterExp,
                c_int(self._camera_id),
                buf,
                c_long(buf_size),
                operation="GetDataAfterExp",
            )

            self._image_buffer = bytes(buf)
            self._exposure_complete.set()
            self._image_ready = True
            logger.debug("image ready")

        except Exception as e:
            logger.error(f"Exposure failed: {e}")
            self._camera_state = CameraState.ERROR
            self._image_ready = False

    def abort_exposure(self) -> None:
        if self._camera_state in (
            CameraState.EXPOSING,
            CameraState.READING,
            CameraState.WAITING,
        ):
            try:
                self._libasicamera2.ASIStopExposure(c_int(self._camera_id))
            except Exception:
                logger.warning("Unable to abort exposure")
                pass
            self._camera_state = CameraState.IDLE
            self._image_ready = False
            self._exposure_complete.set()

    def stop_exposure(self) -> None:
        """Stop exposure — ASI SDK allows reading partial data after stop."""
        self.abort_exposure()

    def pulse_guide(self, direction: int, duration_ms: int) -> None:
        """Send ST4 pulse guide command."""
        if not self._has_st4:
            raise RuntimeError("Camera does not have ST4 port")
        asi_call(
            self._libasicamera2.ASIPulseGuideOn,
            c_int(self._camera_id),
            c_int(direction),
            operation="PulseGuideOn",
        )
        time.sleep(duration_ms / 1000.0)
        asi_call(
            self._libasicamera2.ASIPulseGuideOff,
            c_int(self._camera_id),
            c_int(direction),
            operation="PulseGuideOff",
        )