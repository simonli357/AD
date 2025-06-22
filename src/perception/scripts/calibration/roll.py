#!/usr/bin/env python3
"""
estimate_roll.py  –  Compute camera roll from a single vertical edge.

Inputs (prompted):
    x_top,    y_top    – pixel coords of the top point of a world‑vertical edge
    x_bottom, y_bottom – pixel coords of the bottom point of the same edge

Output:
    roll angle θ in radians and degrees
"""

import math
from typing import Tuple

def estimate_roll(x_top, y_top, x_bot, y_bot):
    dx = x_bot - x_top        #  bottom – top
    dy = y_bot - y_top
    theta_rad = math.atan2(dx, -dy)   # rolls: +clockwise, −counter-clockwise
    return theta_rad, math.degrees(theta_rad)


def main() -> None:
    print("Pixel coordinates of a vertical edge (undistorted image)")
    # x_top  = 270
    # y_top  = 35
    # x_bot  = 277
    # y_bot  = 256
    
    # x_top  = 304
    # y_top  = 61
    # x_bot  = 305
    # y_bot  = 264
    
    x_top  = 304
    y_top  = 58
    x_bot  = 302
    y_bot  = 263
    
    theta_rad, theta_deg = estimate_roll(x_top, y_top, x_bot, y_bot)
    print(f"\nEstimated roll tilt")
    print(f"  θ = {theta_rad:+.6f} rad  ({theta_deg:+.3f} °)")

if __name__ == "__main__":
    main()
