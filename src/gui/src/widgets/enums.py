from enum import Enum


class TerminalType(Enum):
    DEBUG = 1
    SIM = 2
    CONTROL = 3
    CAM = 4
    PATH = 5
    ROSCORE = 6
    TRAFFIC = 7


class OpenGLContextName(Enum):
    MAP = 0
    CAR = 1
    CAM = 2
    BARCA = 3
    GRAPH = 4


class MapData(Enum):
    PNG_WIDTH = 800
    PNG_HEIGHT = 533
    REAL_WORLD_WIDTH = 20.696
    REAL_WORLD_HEIGHT = 13.786
    REAL_X_PER_PIXEL = REAL_WORLD_WIDTH / PNG_WIDTH
    REAL_Y_PER_PIXEL = REAL_WORLD_HEIGHT / PNG_HEIGHT
    CAR_WIDTH = 0.2
    CAR_LENGTH = 0.4
    CAR_HEIGHT = 0.15


class BarcaMapData(Enum):
    REAL_WORLD_WIDTH = 70.0
    REAL_WORLD_HEIGHT = 25.0
    CAR_WIDTH = 0.2
    CAR_LENGTH = 0.4
    CAR_HEIGHT = 0.15


class CameraParams(Enum):
    MIN_WIDTH = 640
    MIN_HEIGHT = 480
    FPS_60 = 0.017  # 60 FPS
    FPS_30 = 0.034  # 30 FPS
    FPS_5 = 0.2     # 5 FPS
    RECORDING_REFRESH_RATE = 0.5  # 1/rate


class NamedColor(Enum):
    RED = (1.0, 0.0, 0.0, 1.0)
    BLUE = (0.0, 0.0, 1.0, 1.0)
    GREEN = (0.0, 1.0, 0.0, 1.0)
    YELLOW = (1.0, 1.0, 0.0, 1.0)
    CYAN = (0.0, 1.0, 1.0, 1.0)
    WHITE = (1.0, 1.0, 1.0, 1.0)
    ORANGE = (1.0, 0.65, 0.0, 1.0)
    INDIGO = (0.29, 0.0, 0.51, 1.0)
    LIGHT_PINK = (1.0, 0.71, 0.76, 1.0)
    STEEL_BLUE = (0.27, 0.51, 0.71, 1.0)
    PURPLE = (0.51, 0.0, 0.51, 1.0)


class RoadObjectsColor(Enum):
    ONEWAY = (1.0, 0.0, 1.0)    # magenta
    HIGHWAYENTRANCE = (0.0, 0.5, 0.0)    # dark green
    STOPSIGN = (1.0, 0.0, 0.0)    # red
    ROUNDABOUT = (0.0, 1.0, 1.0)    # cyan
    PARKING = (0.0, 0.0, 1.0)    # blue
    CROSSWALK = (1.0, 1.0, 0.0)    # yellow
    NOENTRY = (0.0, 1.0, 0.0)    # lime
    HIGHWAYEXIT = (1.0, 0.647, 0.0)    # orange
    PRIORITY = (0.5, 0.5, 0.0)    # olive
    LIGHT = (0.0, 0.5, 0.5)    # teal
    BLOCK = (0.5, 0.0, 0.0)    # maroon
    PEDESTRIAN = (0.0, 0.0, 0.5)    # navy
    CAR = (0.5, 0.0, 0.5)    # purple
    GREENLIGHT = (0.0, 1.0, 0.0)    # bright green
    YELLOWLIGHT = (1.0, 1.0, 0.0)    # yellow
    REDLIGHT = (1.0, 0.0, 0.0)    # red
    SIGN = (0.5, 0.5, 0.5)  # grey
