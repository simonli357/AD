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


class CameraParams(Enum):
    MIN_WIDTH = 640
    MIN_HEIGHT = 480
    FPS_60 = 0.017  # 60FPS
    FPS_30 = 0.034  # 30 FPS
    RECORDING_REFRESH_RATE = 0.25  # 1/rate
