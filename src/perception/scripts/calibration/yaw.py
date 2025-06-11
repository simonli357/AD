#!/usr/bin/env python3
"""
Estimate camera‑to‑car yaw mis‑alignment (ψ).

Inputs
------
fx           : focal length in pixels (from camera intrinsics)
img_width    : image width in pixels
y_left       : metres seen to the *left* of the car centreline at distance L
y_right      : metres seen to the *right* of the car centreline at distance L
L            : forward distance from the camera to the measurement line (metres)
h            : camera height above ground (metres)   [unused in yaw calc]
pitch_deg    : camera pitch down (+) or up (‑) in deg [unused in yaw calc]

Returns
-------
ψ_rad, ψ_deg : yaw error in radians and degrees
"""

import math
from typing import Tuple

def estimate_yaw(fx: float,
                 img_width: float,
                 y_left: float,
                 y_right: float,
                 L: float) -> Tuple[float, float]:
    """Closed‑form half‑angle solution."""
    # horizontal half‑FOV  φ  (for information only)
    phi = math.atan(img_width / (2.0 * fx))

    # yaw error ψ (positive = camera points to the left)
    psi_rad = 0.5 * (math.atan(y_left / L) - math.atan(y_right / L))
    psi_deg = math.degrees(psi_rad)
    return psi_rad, psi_deg


def main() -> None:
    fx          = 607.40564
    img_width   = 640
    y_left      = 0.79
    y_right     = 0.83
    L           = 5*0.3048
    h           = 0.257
    pitch_deg   = 1.85

    psi_rad, psi_deg = estimate_yaw(fx, img_width, y_left, y_right, L)

    print("\nEstimated yaw mis‑alignment")
    print(f"  ψ = {psi_rad:+.6f} rad  ({psi_deg:+.3f} °)")

if __name__ == "__main__":
    main()
