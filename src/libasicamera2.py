import os
from ctypes import (
    CDLL,
    POINTER,
    Structure,
    c_char,
    c_double,
    c_float,
    c_int,
    c_long,
    c_ubyte,
    c_uint,
)
import sys
from typing import List, Optional, Union

from log import get_logger

logger = get_logger()

IS_WINDOWS = sys.platform.startswith("win")
IS_LINUX = sys.platform.startswith("linux")


# ──────────────────────────────────────────────────────────────────
# Enumerations
# ──────────────────────────────────────────────────────────────────
class ASI_BAYER_PATTERN:
    RG = 0
    BG = 1
    GR = 2
    GB = 3


class ASI_IMG_TYPE:
    RAW8 = 0
    RGB24 = 1
    RAW16 = 2
    Y8 = 3
    END = -1


class ASI_GUIDE_DIRECTION:
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3


class ASI_ERROR_CODE:
    SUCCESS = 0
    INVALID_INDEX = 1
    INVALID_ID = 2
    INVALID_CONTROL_TYPE = 3
    CAMERA_CLOSED = 4
    CAMERA_REMOVED = 5
    INVALID_PATH = 6
    INVALID_FILEFORMAT = 7
    INVALID_SIZE = 8
    INVALID_IMGTYPE = 9
    OUTOF_BOUNDARY = 10
    TIMEOUT = 11
    INVALID_SEQUENCE = 12
    BUFFER_TOO_SMALL = 13
    VIDEO_MODE_ACTIVE = 14
    EXPOSURE_IN_PROGRESS = 15
    GENERAL_ERROR = 16
    END = 17


ASI_ERROR_NAMES = {
    0: "Success",
    1: "InvalidIndex",
    2: "InvalidID",
    3: "InvalidControlType",
    4: "CameraClosed",
    5: "CameraRemoved",
    6: "InvalidPath",
    7: "InvalidFileFormat",
    8: "InvalidSize",
    9: "InvalidImgType",
    10: "OutOfBoundary",
    11: "Timeout",
    12: "InvalidSequence",
    13: "BufferTooSmall",
    14: "VideoModeActive",
    15: "ExposureInProgress",
    16: "GeneralError",
}


class ASI_BOOL:
    FALSE = 0
    TRUE = 1


class ASI_CONTROL_TYPE:
    GAIN = 0
    EXPOSURE = 1
    GAMMA = 2
    WB_R = 3
    WB_B = 4
    BRIGHTNESS = 5  # offset
    BANDWIDTHOVERLOAD = 6
    OVERCLOCK = 7
    TEMPERATURE = 8  # 10x actual
    FLIP = 9
    AUTO_MAX_GAIN = 10
    AUTO_MAX_EXP = 11  # microseconds
    AUTO_MAX_BRIGHTNESS = 12
    HARDWARE_BIN = 13
    HIGH_SPEED_MODE = 14
    COOLER_POWER_PERC = 15
    TARGET_TEMP = 16  # not 10x
    COOLER_ON = 17
    MONO_BIN = 18
    FAN_ON = 19
    PATTERN_ADJUST = 20
    ANTI_DEW_HEATER = 21


class ASI_EXPOSURE_STATUS:
    IDLE = 0
    WORKING = 1
    SUCCESS = 2
    FAILED = 3


# ──────────────────────────────────────────────────────────────────
# Structures
# ──────────────────────────────────────────────────────────────────
class ASI_CAMERA_INFO(Structure):
    _fields_ = [
        ("Name", c_char * 64),
        ("CameraID", c_int),
        ("MaxHeight", c_long),
        ("MaxWidth", c_long),
        ("IsColorCam", c_int),
        ("BayerPattern", c_int),
        ("SupportedBins", c_int * 16),
        ("SupportedVideoFormat", c_int * 8),
        ("PixelSize", c_double),
        ("MechanicalShutter", c_int),
        ("ST4Port", c_int),
        ("IsCoolerCam", c_int),
        ("IsUSB3Host", c_int),
        ("IsUSB3Camera", c_int),
        ("ElecPerADU", c_float),
        ("BitDepth", c_int),
        ("IsTriggerCam", c_int),
        ("Unused", c_char * 16),
    ]


class ASI_CONTROL_CAPS(Structure):
    _fields_ = [
        ("Name", c_char * 64),
        ("Description", c_char * 128),
        ("MaxValue", c_long),
        ("MinValue", c_long),
        ("DefaultValue", c_long),
        ("IsAutoSupported", c_int),
        ("IsWritable", c_int),
        ("ControlType", c_int),
        ("Unused", c_char * 32),
    ]


class ASI_ID(Structure):
    _fields_ = [
        ("id", c_ubyte * 8),
    ]


class ASI_SUPPORTED_MODE(Structure):
    _fields_ = [
        ("SupportedCameraMode", c_int * 16),
    ]


# ──────────────────────────────────────────────────────────────────
# Error handling
# ──────────────────────────────────────────────────────────────────
def asi_error_string(error_code: int) -> str:
    return f"ASI_{ASI_ERROR_NAMES.get(error_code, f'Unknown({error_code})')}"


class ASIError(Exception):
    def __init__(self, error_code: int, operation: str = ""):
        self.error_code = error_code
        self.error_string = asi_error_string(error_code)
        self.operation = operation
        message = f"ASI error {error_code}: {self.error_string}"
        if operation:
            message = f"{operation}: {message}"
        super().__init__(message)


def asi_call(func, *args, operation: str = ""):
    error = func(*args)
    if error != 0:
        raise ASIError(error, operation or func.__name__)
    return error


# ──────────────────────────────────────────────────────────────────
# Argtypes / restypes
# ──────────────────────────────────────────────────────────────────
_PE = c_int  # ASI_ERROR_CODE return


def _configure_argtypes(lib: Union[CDLL, "WinDLL"]) -> None:
    """Declare argtypes and restype for every ASICamera2 function we call.

    Without these, ctypes on 64-bit Windows silently mangles pointer
    arguments, leading to intermittent segfaults inside the native DLL.
    """

    # --- Camera discovery ---
    lib.ASIGetNumOfConnectedCameras.argtypes = []
    lib.ASIGetNumOfConnectedCameras.restype = c_int

    lib.ASIGetCameraProperty.argtypes = [POINTER(ASI_CAMERA_INFO), c_int]
    lib.ASIGetCameraProperty.restype = _PE

    # --- Open / Init / Close ---
    lib.ASIOpenCamera.argtypes = [c_int]
    lib.ASIOpenCamera.restype = _PE

    lib.ASIInitCamera.argtypes = [c_int]
    lib.ASIInitCamera.restype = _PE

    lib.ASICloseCamera.argtypes = [c_int]
    lib.ASICloseCamera.restype = _PE

    # --- Controls ---
    lib.ASIGetNumOfControls.argtypes = [c_int, POINTER(c_int)]
    lib.ASIGetNumOfControls.restype = _PE

    lib.ASIGetControlCaps.argtypes = [c_int, c_int, POINTER(ASI_CONTROL_CAPS)]
    lib.ASIGetControlCaps.restype = _PE

    lib.ASIGetControlValue.argtypes = [c_int, c_int, POINTER(c_long), POINTER(c_int)]
    lib.ASIGetControlValue.restype = _PE

    lib.ASISetControlValue.argtypes = [c_int, c_int, c_long, c_int]
    lib.ASISetControlValue.restype = _PE

    # --- ROI ---
    lib.ASISetROIFormat.argtypes = [c_int, c_int, c_int, c_int, c_int]
    lib.ASISetROIFormat.restype = _PE

    lib.ASIGetROIFormat.argtypes = [c_int, POINTER(c_int), POINTER(c_int), POINTER(c_int), POINTER(c_int)]
    lib.ASIGetROIFormat.restype = _PE

    lib.ASISetStartPos.argtypes = [c_int, c_int, c_int]
    lib.ASISetStartPos.restype = _PE

    lib.ASIGetStartPos.argtypes = [c_int, POINTER(c_int), POINTER(c_int)]
    lib.ASIGetStartPos.restype = _PE

    # --- Exposure (snap mode) ---
    lib.ASIStartExposure.argtypes = [c_int, c_int]
    lib.ASIStartExposure.restype = _PE

    lib.ASIStopExposure.argtypes = [c_int]
    lib.ASIStopExposure.restype = _PE

    lib.ASIGetExpStatus.argtypes = [c_int, POINTER(c_int)]
    lib.ASIGetExpStatus.restype = _PE

    lib.ASIGetDataAfterExp.argtypes = [c_int, POINTER(c_ubyte), c_long]
    lib.ASIGetDataAfterExp.restype = _PE

    # --- Pulse guide ---
    lib.ASIPulseGuideOn.argtypes = [c_int, c_int]
    lib.ASIPulseGuideOn.restype = _PE

    lib.ASIPulseGuideOff.argtypes = [c_int, c_int]
    lib.ASIPulseGuideOff.restype = _PE

    # --- SDK version ---
    lib.ASIGetSDKVersion.argtypes = []
    lib.ASIGetSDKVersion.restype = c_char * 128

    logger.debug("ASICamera2 argtypes configured for all functions")


# ──────────────────────────────────────────────────────────────────
# Library loader
# ──────────────────────────────────────────────────────────────────
def load_asi_library(library: str) -> Optional[CDLL]:
    """Load the ASICamera2 library, setting up DLL search directories on Windows."""
    try:
        if IS_WINDOWS:
            from ctypes import WinDLL
            lib = WinDLL(library)
        else:
            lib = CDLL(library)

        logger.debug(f"loaded ASICamera2 library from {library}")
        _configure_argtypes(lib)
        return lib
    except OSError as e:
        logger.error(f"Failed to load ASICamera2 library from {library}: {e}")
        return None
