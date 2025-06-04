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

def estimate_roll(x_top: float, y_top: float,
                  x_bot: float, y_bot: float) -> Tuple[float, float]:
    # slope in pixel space  (Δx / Δy)
    dx = x_top - x_bot
    dy = y_top - y_bot
    theta_rad = math.atan2(dx, dy)      # +ve = clockwise tilt
    theta_deg = math.degrees(theta_rad)
    return theta_rad, theta_deg


def main() -> None:
    print("Pixel coordinates of a vertical edge (undistorted image)")
    x_top  = 299
    y_top  = 0
    x_bot  = 301
    y_bot  = 322

    theta_rad, theta_deg = estimate_roll(x_top, y_top, x_bot, y_bot)
    print(f"\nEstimated roll tilt")
    print(f"  θ = {theta_rad:+.6f} rad  ({theta_deg:+.3f} °)")

if __name__ == "__main__":
    main()
