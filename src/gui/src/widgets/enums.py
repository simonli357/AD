from enum import Enum


class TerminalType(Enum):
    DEBUG = 1
    SIM = 2
    CONTROL = 3
    CAM = 4
    PATH = 5
    ROSCORE = 6


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
    MAP_CENTER_X = 4.0
    MAP_CENTER_Y = 2.0
    MAP_WIDTH = 70.0
    MAP_HEIGHT = 25.0
    CAR_WIDTH = 0.2
    CAR_LENGTH = 0.4
    CAR_HEIGHT = 0.15


class CameraParams(Enum):
    MIN_WIDTH = 640
    MIN_HEIGHT = 480
    FPS_60 = 0.017  # 60 FPS
    FPS_30 = 0.034  # 30 FPS
    FPS_10 = 0.100  # 10 FPS
    FPS_5 = 0.200   # 5 FPS
    RECORDING_REFRESH_RATE = 2.0  # Capture 1 frame every 2 seconds if recording
