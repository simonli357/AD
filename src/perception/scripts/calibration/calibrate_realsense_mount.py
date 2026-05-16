#!/usr/bin/env python3
"""
Guided RealSense D435i mount calibration using a printed ChArUco target.

The script is intentionally self-contained so an operator can run:

    python3 calibrate_realsense_mount.py wizard

from this directory and follow the prompts.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import yaml
except Exception:  # pragma: no cover - exercised by the check command
    yaml = None


SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR / "assets"
RUNS_DIR = SCRIPT_DIR / "runs"
DEFAULT_CONSTANTS_PATH = SCRIPT_DIR.parents[3] / "src" / "utils" / "include" / "utils" / "constants.h"
DEFAULT_TUNABLES_PATH = SCRIPT_DIR.parents[3] / "src" / "control" / "config" / "tunable_params.yaml"

DEFAULT_SPEC: Dict[str, Any] = {
    "version": 1,
    "dictionary": "DICT_4X4_100",
    "squares_x": 8,
    "squares_y": 6,
    "square_size_m": 0.030,
    "marker_size_m": 0.022,
    "dpi": 300,
    "board_width_m": 0.240,
    "board_height_m": 0.180,
    "scale_bar_m": 0.100,
    "pages": {
        "letter": {
            "name": "Letter landscape",
            "width_mm": 279.4,
            "height_mm": 215.9,
            "file": "charuco_ground_target_letter_300dpi.png",
        },
        "a4": {
            "name": "A4 landscape",
            "width_mm": 297.0,
            "height_mm": 210.0,
            "file": "charuco_ground_target_a4_300dpi.png",
        },
        "a2_large2x": {
            "name": "A2 landscape, 2x floor target",
            "width_mm": 594.0,
            "height_mm": 420.0,
            "file": "charuco_ground_target_a2_large_2x_300dpi.png",
            "target_scale": 2.0,
        },
    },
}

MIN_VALID_FRAMES = 15
MIN_CHARUCO_CORNERS = 12
DEFAULT_FRAME_COUNT = 40


@dataclass
class Detection:
    marker_corners: List[np.ndarray]
    marker_ids: Optional[np.ndarray]
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray

    @property
    def corner_count(self) -> int:
        return int(len(self.charuco_ids))

    @property
    def marker_count(self) -> int:
        if self.marker_ids is None:
            return 0
        return int(len(self.marker_ids))


@dataclass
class Observation:
    image_name: str
    charuco_id: int
    object_point: np.ndarray
    image_point: np.ndarray


def require_yaml() -> Any:
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install it with: python3 -m pip install pyyaml")
    return yaml


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def m_to_px(meters: float, dpi: int) -> int:
    return mm_to_px(meters * 1000.0, dpi)


def now_stamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def load_yaml(path: Path) -> Dict[str, Any]:
    y = require_yaml()
    with path.open("r", encoding="utf-8") as fh:
        data = y.safe_load(fh) or {}
    return data


def write_yaml(path: Path, data: Dict[str, Any]) -> None:
    y = require_yaml()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        y.safe_dump(data, fh, sort_keys=False)


def load_target_spec(path: Optional[Path] = None) -> Dict[str, Any]:
    path = path or ASSETS_DIR / "target_spec.yaml"
    if path.exists():
        spec = load_yaml(path)
    else:
        spec = dict(DEFAULT_SPEC)
    return normalize_target_spec(spec)


def normalize_target_spec(spec: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(DEFAULT_SPEC)
    merged.update(spec or {})
    merged["pages"] = {**DEFAULT_SPEC["pages"], **(spec or {}).get("pages", {})}
    merged["board_width_m"] = float(merged["squares_x"]) * float(merged["square_size_m"])
    merged["board_height_m"] = float(merged["squares_y"]) * float(merged["square_size_m"])
    return merged


def aruco_module() -> Any:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV was built without aruco. Install an OpenCV contrib build.")
    return cv2.aruco


def aruco_dictionary(name: str) -> Any:
    aruco = aruco_module()
    if not hasattr(aruco, name):
        raise RuntimeError(f"Unknown ArUco dictionary {name!r}")
    code = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(code)
    return aruco.Dictionary_get(code)


def detector_parameters() -> Any:
    aruco = aruco_module()
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def create_charuco_board(spec: Dict[str, Any]) -> Any:
    aruco = aruco_module()
    dictionary = aruco_dictionary(str(spec["dictionary"]))
    sx = int(spec["squares_x"])
    sy = int(spec["squares_y"])
    square = float(spec["square_size_m"])
    marker = float(spec["marker_size_m"])
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(sx, sy, square, marker, dictionary)
    return aruco.CharucoBoard((sx, sy), square, marker, dictionary)


def draw_board(spec: Dict[str, Any], width_px: int, height_px: int) -> np.ndarray:
    board = create_charuco_board(spec)
    if hasattr(board, "generateImage"):
        image = board.generateImage((width_px, height_px))
    else:
        image = board.draw((width_px, height_px))
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def charuco_id_to_target_point(
    corner_id: int,
    spec: Dict[str, Any],
    print_scale: float = 1.0,
    target_placement: str = "ground",
) -> np.ndarray:
    """Return a ChArUco corner in the vehicle-aligned target frame."""
    squares_x = int(spec["squares_x"])
    square = float(spec["square_size_m"]) * float(print_scale)
    board_width = squares_x * square
    board_height = int(spec["squares_y"]) * square
    inner_cols = squares_x - 1
    col = int(corner_id) % inner_cols
    row = int(corner_id) // inner_cols
    board_x = (col + 1) * square
    board_y = (row + 1) * square
    y_left = board_width / 2.0 - board_x
    if target_placement == "ground":
        x_forward = board_height / 2.0 - board_y
        return np.array([x_forward, y_left, 0.0], dtype=np.float32)
    if target_placement == "vertical-front":
        z_up = board_height / 2.0 - board_y
        return np.array([0.0, y_left, z_up], dtype=np.float32)
    raise ValueError(f"Unsupported target placement: {target_placement}")


def object_points_for_ids(
    ids: np.ndarray,
    spec: Dict[str, Any],
    print_scale: float,
    target_placement: str = "ground",
) -> np.ndarray:
    return np.array(
        [charuco_id_to_target_point(int(i), spec, print_scale, target_placement) for i in ids.reshape(-1)],
        dtype=np.float32,
    )


def detect_charuco(image: np.ndarray, spec: Dict[str, Any]) -> Detection:
    aruco = aruco_module()
    board = create_charuco_board(spec)
    dictionary = aruco_dictionary(str(spec["dictionary"]))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=detector_parameters())
    empty_corners = np.empty((0, 2), dtype=np.float32)
    empty_ids = np.empty((0,), dtype=np.int32)
    if ids is None or len(ids) == 0:
        return Detection([], None, empty_corners, empty_ids)
    result = aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if result is None:
        return Detection(corners, ids, empty_corners, empty_ids)
    _, charuco_corners, charuco_ids = result
    if charuco_corners is None or charuco_ids is None:
        return Detection(corners, ids, empty_corners, empty_ids)
    return Detection(
        marker_corners=corners,
        marker_ids=ids,
        charuco_corners=np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2),
        charuco_ids=np.asarray(charuco_ids, dtype=np.int32).reshape(-1),
    )


def draw_detection_preview(image: np.ndarray, detection: Detection, min_corners: int) -> np.ndarray:
    aruco = aruco_module()
    preview = image.copy()
    if detection.marker_ids is not None and detection.marker_count:
        aruco.drawDetectedMarkers(preview, detection.marker_corners, detection.marker_ids)
    if detection.corner_count:
        corners = detection.charuco_corners.reshape(-1, 1, 2)
        ids = detection.charuco_ids.reshape(-1, 1)
        aruco.drawDetectedCornersCharuco(preview, corners, ids)
    color = (30, 180, 30) if detection.corner_count >= min_corners else (30, 30, 220)
    cv2.putText(
        preview,
        f"ChArUco corners: {detection.corner_count} / required {min_corners}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )
    return preview


def adjust_intrinsics_for_rotate180(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    width: int,
    height: int,
) -> Tuple[np.ndarray, np.ndarray]:
    adjusted_k = np.array(camera_matrix, dtype=np.float64, copy=True)
    adjusted_k[0, 2] = float(width - 1) - adjusted_k[0, 2]
    adjusted_k[1, 2] = float(height - 1) - adjusted_k[1, 2]
    adjusted_d = np.array(dist_coeffs, dtype=np.float64, copy=True).reshape(-1)
    if adjusted_d.size >= 4:
        adjusted_d[2] *= -1.0
        adjusted_d[3] *= -1.0
    return adjusted_k, adjusted_d


def apply_runtime_flip(image: np.ndarray, runtime_flip: str) -> np.ndarray:
    if runtime_flip == "none":
        return image
    if runtime_flip == "rotate180":
        return cv2.rotate(image, cv2.ROTATE_180)
    raise ValueError(f"Unsupported runtime flip: {runtime_flip}")


def compose_repo_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    rz = np.array(
        [
            [math.cos(-yaw), -math.sin(-yaw), 0.0],
            [math.sin(-yaw), math.cos(-yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    ry = np.array(
        [
            [math.cos(-pitch), 0.0, math.sin(-pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(-pitch), 0.0, math.cos(-pitch)],
        ],
        dtype=np.float64,
    )
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(-roll), -math.sin(-roll)],
            [0.0, math.sin(-roll), math.cos(-roll)],
        ],
        dtype=np.float64,
    )
    rs = axis_switch_matrix()
    return rs @ (rz @ (ry @ rx))


def axis_switch_matrix() -> np.ndarray:
    return np.array(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )


def extract_repo_euler(rotation_vehicle_to_camera: np.ndarray) -> Tuple[float, float, float]:
    """Extract roll, pitch, yaw from R = Rs * Rz(-yaw) * Ry(-pitch) * Rx(-roll)."""
    a = axis_switch_matrix().T @ np.asarray(rotation_vehicle_to_camera, dtype=np.float64)
    sy = math.hypot(float(a[0, 0]), float(a[1, 0]))
    if sy > 1e-9:
        z_angle = math.atan2(float(a[1, 0]), float(a[0, 0]))
        y_angle = math.atan2(float(-a[2, 0]), sy)
        x_angle = math.atan2(float(a[2, 1]), float(a[2, 2]))
    else:
        z_angle = math.atan2(float(-a[0, 1]), float(a[1, 1]))
        y_angle = math.atan2(float(-a[2, 0]), sy)
        x_angle = 0.0
    roll = -x_angle
    pitch = -y_angle
    yaw = -z_angle
    return roll, pitch, yaw


def matrix_to_list(matrix: np.ndarray) -> List[List[float]]:
    return [[float(v) for v in row] for row in np.asarray(matrix)]


def vector_to_list(vector: np.ndarray) -> List[float]:
    return [float(v) for v in np.asarray(vector).reshape(-1)]


def intrinsics_to_camera_matrix(intrinsics: Any) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    k = np.array(
        [
            [float(intrinsics.fx), 0.0, float(intrinsics.ppx)],
            [0.0, float(intrinsics.fy), float(intrinsics.ppy)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    d = np.array(list(intrinsics.coeffs), dtype=np.float64).reshape(-1)
    meta = {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "cx": float(intrinsics.ppx),
        "cy": float(intrinsics.ppy),
        "distortion_model": str(intrinsics.model),
        "dist_coeffs_raw": vector_to_list(d),
    }
    return k, d, meta


def save_png_with_dpi(path: Path, image_bgr: np.ndarray, dpi: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image

        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        Image.fromarray(rgb).save(path, dpi=(dpi, dpi))
    except Exception:
        cv2.imwrite(str(path), image_bgr)


def put_centered_text(
    image: np.ndarray,
    text: str,
    center: Tuple[int, int],
    scale: float,
    color: Tuple[int, int, int] = (0, 0, 0),
    thickness: int = 2,
) -> None:
    size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x = int(center[0] - size[0] / 2)
    y = int(center[1] + size[1] / 2)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def draw_arrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int], label: str) -> None:
    cv2.arrowedLine(image, start, end, (0, 0, 0), 4, cv2.LINE_AA, tipLength=0.25)
    mid = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
    put_centered_text(image, label, (mid[0], mid[1] - 12), 0.7, thickness=2)


def draw_target_page(spec: Dict[str, Any], page_key: str, output_path: Path) -> None:
    page = spec["pages"][page_key]
    dpi = int(spec["dpi"])
    target_scale = float(page.get("target_scale", 1.0))
    page_w = mm_to_px(float(page["width_mm"]), dpi)
    page_h = mm_to_px(float(page["height_mm"]), dpi)
    board_w = m_to_px(float(spec["board_width_m"]) * target_scale, dpi)
    board_h = m_to_px(float(spec["board_height_m"]) * target_scale, dpi)
    board_left = (page_w - board_w) // 2
    board_top = (page_h - board_h) // 2
    board_right = board_left + board_w
    board_bottom = board_top + board_h

    canvas = np.full((page_h, page_w, 3), 255, dtype=np.uint8)
    board_image = draw_board(spec, board_w, board_h)
    canvas[board_top:board_bottom, board_left:board_right] = board_image

    center_x = board_left + board_w // 2
    center_y = board_top + board_h // 2
    tick = mm_to_px(6.0, dpi)
    cv2.line(canvas, (center_x, board_top - tick), (center_x, board_top - 1), (0, 0, 0), 3)
    cv2.line(canvas, (center_x, board_bottom + 1), (center_x, board_bottom + tick), (0, 0, 0), 3)
    cv2.line(canvas, (board_left - tick, center_y), (board_left - 1, center_y), (0, 0, 0), 3)
    cv2.line(canvas, (board_right + 1, center_y), (board_right + tick, center_y), (0, 0, 0), 3)
    put_centered_text(canvas, "TARGET CENTER LINES", (center_x, board_bottom + mm_to_px(10.0, dpi)), 0.55, thickness=1)

    x_arrow_start = (center_x, max(mm_to_px(13.0, dpi), board_top - mm_to_px(3.0, dpi)))
    x_arrow_end = (center_x, max(mm_to_px(5.0, dpi), board_top - mm_to_px(13.0, dpi)))
    draw_arrow(canvas, x_arrow_start, x_arrow_end, "+X CAR FORWARD")

    y_arrow_start = (max(mm_to_px(14.0, dpi), board_left - mm_to_px(4.0, dpi)), center_y)
    y_arrow_end = (max(mm_to_px(4.0, dpi), board_left - mm_to_px(15.0, dpi)), center_y)
    draw_arrow(canvas, y_arrow_start, y_arrow_end, "+Y CAR LEFT")

    scale_len = m_to_px(float(spec["scale_bar_m"]) * target_scale, dpi)
    scale_y = page_h - mm_to_px(6.0, dpi)
    scale_x0 = center_x - scale_len // 2
    scale_x1 = scale_x0 + scale_len
    cv2.line(canvas, (scale_x0, scale_y), (scale_x1, scale_y), (0, 0, 0), 5)
    cv2.line(canvas, (scale_x0, scale_y - tick // 2), (scale_x0, scale_y + tick // 2), (0, 0, 0), 4)
    cv2.line(canvas, (scale_x1, scale_y - tick // 2), (scale_x1, scale_y + tick // 2), (0, 0, 0), 4)
    if abs(target_scale - 1.0) < 1e-9:
        scale_label = "100 mm SCALE CHECK"
    else:
        scale_label = f"100 mm BASE SCALE - EXPECT {target_scale * 100.0:.0f} mm"
    put_centered_text(canvas, scale_label, (center_x, scale_y - mm_to_px(5.0, dpi)), 0.55, thickness=1)

    save_png_with_dpi(output_path, canvas, dpi)


def make_targets() -> Dict[str, Any]:
    spec = normalize_target_spec(dict(DEFAULT_SPEC))
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    for page_key, page in spec["pages"].items():
        draw_target_page(spec, page_key, ASSETS_DIR / str(page["file"]))
    write_yaml(ASSETS_DIR / "target_spec.yaml", spec)
    return spec


def dependency_status() -> List[Tuple[str, bool, str]]:
    status: List[Tuple[str, bool, str]] = []
    status.append(("numpy", True, np.__version__))
    status.append(("opencv", True, cv2.__version__))
    aruco_ok = (
        hasattr(cv2, "aruco")
        and hasattr(cv2.aruco, "interpolateCornersCharuco")
        and (hasattr(cv2.aruco, "CharucoBoard_create") or hasattr(cv2.aruco, "CharucoBoard"))
    )
    status.append(("opencv aruco/charuco", aruco_ok, "ok" if aruco_ok else "missing cv2.aruco"))
    status.append(("pyyaml", yaml is not None, "ok" if yaml is not None else "missing"))
    try:
        import PIL  # noqa: F401

        status.append(("pillow", True, "ok"))
    except Exception as exc:
        status.append(("pillow", False, f"{type(exc).__name__}: {exc}"))
    try:
        import pyrealsense2 as rs  # noqa: F401

        status.append(("pyrealsense2", True, "ok"))
    except Exception as exc:
        status.append(("pyrealsense2", False, f"{type(exc).__name__}: {exc}"))
    return status


def cmd_check(_: argparse.Namespace) -> int:
    print("Calibration dependency check")
    failed = False
    for name, ok, detail in dependency_status():
        print(f"  {'OK  ' if ok else 'MISS'} {name}: {detail}")
        failed = failed or not ok
    if failed:
        print("\nInstall missing Python packages with:")
        print("  python3 -m pip install -r src/perception/scripts/calibration/requirements-calibration.txt")
        print("OpenCV must include the contrib aruco module.")
        return 1
    return 0


def cmd_make_target(_: argparse.Namespace) -> int:
    spec = make_targets()
    print("Generated calibration target assets:")
    for page in spec["pages"].values():
        print(f"  {ASSETS_DIR / str(page['file'])}")
    print(f"  {ASSETS_DIR / 'target_spec.yaml'}")
    print("For floor calibration, prefer the A2 2x target.")
    print("Print at Actual size / 100% / no fit to page.")
    return 0


def resolve_run_dir(path_text: Optional[str]) -> Path:
    if path_text:
        return Path(path_text).expanduser().resolve()
    if not RUNS_DIR.exists():
        raise RuntimeError("No runs directory exists yet. Run capture or wizard first.")
    candidates = sorted([p for p in RUNS_DIR.iterdir() if p.is_dir()])
    if not candidates:
        raise RuntimeError("No calibration runs found. Run capture or wizard first.")
    return candidates[-1].resolve()


def create_run_dir(path_text: Optional[str]) -> Path:
    if path_text:
        run_dir = Path(path_text).expanduser().resolve()
    else:
        run_dir = RUNS_DIR / now_stamp()
    (run_dir / "images").mkdir(parents=True, exist_ok=True)
    (run_dir / "detections").mkdir(parents=True, exist_ok=True)
    return run_dir


def ask_float(prompt: str, default: float) -> float:
    text = input(f"{prompt} [{default:g}]: ").strip()
    if not text:
        return float(default)
    return float(text)


def ask_int(prompt: str, default: int) -> int:
    text = input(f"{prompt} [{default:d}]: ").strip()
    if not text:
        return int(default)
    return int(text)


def capture_realsense(args: argparse.Namespace) -> Path:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        raise RuntimeError(
            "pyrealsense2 is required for direct RealSense capture. "
            "Install it, or capture images separately and run solve on an existing run folder."
        ) from exc

    spec = load_target_spec()
    run_dir = create_run_dir(args.run_dir)
    print_scale = float(args.print_scale_mm) / 100.0
    runtime_flip = str(args.runtime_flip)
    min_corners = int(args.min_corners)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        int(args.color_width),
        int(args.color_height),
        rs.format.bgr8,
        int(args.color_fps),
    )

    print("\nStarting RealSense color stream...")
    profile = pipeline.start(config)
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = stream.get_intrinsics()
    camera_matrix, dist_coeffs, intr_meta = intrinsics_to_camera_matrix(intrinsics)
    width = int(intr_meta["width"])
    height = int(intr_meta["height"])
    if runtime_flip == "rotate180":
        camera_matrix, dist_coeffs = adjust_intrinsics_for_rotate180(camera_matrix, dist_coeffs, width, height)

    dataset: Dict[str, Any] = {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "runtime_flip": runtime_flip,
        "print_scale_mm": float(args.print_scale_mm),
        "print_scale": print_scale,
        "target_spec": spec,
        "camera": {
            **intr_meta,
            "camera_matrix": matrix_to_list(camera_matrix),
            "dist_coeffs": vector_to_list(dist_coeffs),
            "intrinsics_note": "camera_matrix/dist_coeffs are adjusted for runtime_flip",
        },
        "frames": [],
    }

    saved = 0
    last_auto_save = 0.0
    window_name = "RealSense mount calibration capture"
    if not args.no_window:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("Capture controls:")
    print("  SPACE: save a frame when the board is detected")
    print("  q: finish")
    print("Keep the car and target fixed while capturing.")
    if args.max_seconds:
        print(f"Capture will stop after {float(args.max_seconds):.1f} seconds even if fewer frames are saved.")

    try:
        for _ in range(int(args.warmup)):
            pipeline.wait_for_frames()
        started_at = time.time()
        while saved < int(args.frames):
            if args.max_seconds and time.time() - started_at >= float(args.max_seconds):
                print("Reached capture time limit.")
                break
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            raw = np.asanyarray(color_frame.get_data())
            image = apply_runtime_flip(raw, runtime_flip)
            detection = detect_charuco(image, spec)
            preview = draw_detection_preview(image, detection, min_corners)

            key = -1
            if not args.no_window:
                cv2.imshow(window_name, preview)
                key = cv2.waitKey(1) & 0xFF
            should_save = key == 32
            if args.auto:
                now = time.time()
                should_save = detection.corner_count >= min_corners and now - last_auto_save >= float(args.interval)
            if key in (ord("q"), ord("Q")):
                break
            if should_save:
                if detection.corner_count < min_corners:
                    print(f"Not saved: only {detection.corner_count} ChArUco corners detected.")
                    continue
                image_rel = Path("images") / f"frame_{saved:03d}.png"
                preview_rel = Path("detections") / f"frame_{saved:03d}_detection.png"
                cv2.imwrite(str(run_dir / image_rel), image)
                cv2.imwrite(str(run_dir / preview_rel), preview)
                dataset["frames"].append(
                    {
                        "file": str(image_rel),
                        "detection_preview": str(preview_rel),
                        "charuco_corners": int(detection.corner_count),
                        "aruco_markers": int(detection.marker_count),
                    }
                )
                saved += 1
                last_auto_save = time.time()
                print(f"Saved {saved}/{args.frames}: {image_rel}")
    finally:
        pipeline.stop()
        if not args.no_window:
            cv2.destroyWindow(window_name)

    write_yaml(run_dir / "dataset.yaml", dataset)
    print(f"\nDataset written to {run_dir}")
    if saved < MIN_VALID_FRAMES:
        print(f"Warning: saved {saved} frames. Recommended minimum is {MIN_VALID_FRAMES}.")
    return run_dir


def cmd_capture(args: argparse.Namespace) -> int:
    run_dir = capture_realsense(args)
    print(f"Next step: python3 {Path(__file__).name} solve {run_dir}")
    return 0


def load_dataset(run_dir: Path) -> Dict[str, Any]:
    dataset_path = run_dir / "dataset.yaml"
    if not dataset_path.exists():
        raise RuntimeError(f"Missing dataset.yaml in {run_dir}")
    return load_yaml(dataset_path)


def collect_observations(
    run_dir: Path,
    dataset: Dict[str, Any],
    min_corners: int,
) -> Tuple[List[Observation], Dict[str, Dict[str, Any]]]:
    spec = normalize_target_spec(dataset.get("target_spec") or load_target_spec())
    print_scale = float(dataset.get("print_scale", 1.0))
    observations: List[Observation] = []
    frame_stats: Dict[str, Dict[str, Any]] = {}
    detections_dir = run_dir / "detections"
    detections_dir.mkdir(exist_ok=True)

    frames = dataset.get("frames") or []
    for frame in frames:
        rel_file = Path(str(frame["file"]))
        image_path = run_dir / rel_file
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            frame_stats[str(rel_file)] = {"accepted": False, "reason": "could not read image"}
            continue
        detection = detect_charuco(image, spec)
        preview = draw_detection_preview(image, detection, min_corners)
        preview_path = detections_dir / f"{image_path.stem}_detection.png"
        cv2.imwrite(str(preview_path), preview)
        accepted = detection.corner_count >= min_corners
        frame_stats[str(rel_file)] = {
            "accepted": accepted,
            "charuco_corners": int(detection.corner_count),
            "aruco_markers": int(detection.marker_count),
        }
        if not accepted:
            continue
        object_points = object_points_for_ids(detection.charuco_ids, spec, print_scale)
        for charuco_id, object_point, image_point in zip(
            detection.charuco_ids.reshape(-1),
            object_points,
            detection.charuco_corners.reshape(-1, 2),
        ):
            observations.append(
                Observation(
                    image_name=str(rel_file),
                    charuco_id=int(charuco_id),
                    object_point=np.asarray(object_point, dtype=np.float32),
                    image_point=np.asarray(image_point, dtype=np.float32),
                )
            )
    return observations, frame_stats


def parse_numeric_array_from_constants(path: Path, symbol: str) -> Optional[List[float]]:
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"{re.escape(symbol)}\s*=\s*\{{([^}}]+)\}}", text, flags=re.MULTILINE)
    if not match:
        return None
    raw = match.group(1)
    values: List[float] = []
    for part in raw.split(","):
        expr = part.split("//", 1)[0].strip()
        if not expr:
            continue
        try:
            values.append(float(eval(expr, {"__builtins__": {}}, {"M_PI": math.pi, "PI": math.pi, "pi": math.pi})))
        except Exception:
            return None
    return values


def parse_realsense_tf_from_tunables(path: Path) -> Optional[List[float]]:
    if yaml is None or not path.exists():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    values = data.get("real", {}).get("realsense_tf_real")
    if not isinstance(values, list) or len(values) < 6:
        return None
    try:
        return [float(value) for value in values[:6]]
    except (TypeError, ValueError):
        return None


def read_existing_constants(path: Path, tunables_path: Path = DEFAULT_TUNABLES_PATH) -> Tuple[List[float], List[float]]:
    camera = parse_numeric_array_from_constants(path, "CAMERA_PARAMS_REAL")
    tf = parse_realsense_tf_from_tunables(tunables_path)
    if camera is None or len(camera) < 4:
        camera = [607.40564, 607.05829, 322.97223, 244.39398]
    if tf is None or len(tf) < 6:
        tf = [-0.090, -0.032, 0.260, -0.009774, 1.85 * math.pi / 180.0, 0.011]
    return camera[:4], tf[:6]


def format_cpp_array(values: Sequence[float]) -> str:
    return "{" + ", ".join(f"{float(v):+.9f}" for v in values) + "}"


def write_constants_patch(
    path: Path,
    camera_params: Sequence[float],
    tf_params: Sequence[float],
    result: Dict[str, Any],
) -> None:
    roll, pitch, yaw = tf_params[3], tf_params[4], tf_params[5]
    text = f"""// Copy CAMERA_PARAMS_REAL into src/utils/include/utils/constants.h.
// Copy realsense_tf_real into src/control/config/tunable_params.yaml.
// The script does not edit source files automatically.

static constexpr std::array<double, 4> CAMERA_PARAMS_REAL = {format_cpp_array(camera_params)};

real:
  realsense_tf_real: [{", ".join(f"{float(v):+.9f}" for v in tf_params)}] # x, y, z, roll, pitch, yaw

// Angles:
//   roll  = {roll:+.9f} rad ({math.degrees(roll):+.6f} deg)
//   pitch = {pitch:+.9f} rad ({math.degrees(pitch):+.6f} deg)
//   yaw   = {yaw:+.9f} rad ({math.degrees(yaw):+.6f} deg)
//
// Reprojection:
//   median = {result['reprojection_error_px']['median']:.4f} px
//   mean   = {result['reprojection_error_px']['mean']:.4f} px
//   max    = {result['reprojection_error_px']['max']:.4f} px
"""
    path.write_text(text, encoding="utf-8")


def runtime_camera_params_from_constants(
    constants_camera: Sequence[float],
    runtime_flip: str,
    runtime_width: int,
    runtime_height: int,
) -> List[float]:
    fx, fy, cx, cy = [float(v) for v in constants_camera[:4]]
    if runtime_flip == "rotate180":
        cx = float(runtime_width - 1) - cx
        cy = float(runtime_height - 1) - cy
    return [fx, fy, cx, cy]


def solve_run(
    run_dir: Path,
    min_corners: int,
    constants_path: Path,
    tunables_path: Path,
    runtime_width: int = 640,
    runtime_height: int = 480,
) -> Dict[str, Any]:
    dataset = load_dataset(run_dir)
    camera = dataset.get("camera") or {}
    if "camera_matrix" not in camera or "dist_coeffs" not in camera:
        raise RuntimeError("dataset.yaml is missing camera_matrix/dist_coeffs")

    camera_matrix = np.asarray(camera["camera_matrix"], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.asarray(camera["dist_coeffs"], dtype=np.float64).reshape(-1)
    observations, frame_stats = collect_observations(run_dir, dataset, min_corners)
    valid_frames = len({obs.image_name for obs in observations})
    if len(observations) < 6:
        raise RuntimeError(f"Need at least 6 ChArUco observations; found {len(observations)}")

    object_points = np.array([obs.object_point for obs in observations], dtype=np.float32).reshape(-1, 3)
    image_points = np.array([obs.image_point for obs in observations], dtype=np.float32).reshape(-1, 2)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        iterationsCount=200,
        reprojectionError=3.0,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("cv2.solvePnPRansac failed")
    if inliers is None or len(inliers) < 6:
        inlier_object_points = object_points
        inlier_image_points = image_points
    else:
        inlier_idx = inliers.reshape(-1)
        inlier_object_points = object_points[inlier_idx]
        inlier_image_points = image_points[inlier_idx]

    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            inlier_object_points,
            inlier_image_points,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
        )
    else:
        _, rvec, tvec = cv2.solvePnP(
            inlier_object_points,
            inlier_image_points,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    rotation, _ = cv2.Rodrigues(rvec)
    roll, pitch, yaw = extract_repo_euler(rotation)
    existing_camera, existing_tf = read_existing_constants(constants_path, tunables_path)
    runtime_flip = str(dataset.get("runtime_flip", "unknown"))
    new_camera = runtime_camera_params_from_constants(
        existing_camera,
        runtime_flip,
        int(runtime_width),
        int(runtime_height),
    )
    new_tf = [
        float(existing_tf[0]),
        float(existing_tf[1]),
        float(existing_tf[2]),
        float(roll),
        float(pitch),
        float(yaw),
    ]
    warnings: List[str] = []
    if valid_frames < MIN_VALID_FRAMES:
        warnings.append(f"Only {valid_frames} valid frames; recommended minimum is {MIN_VALID_FRAMES}.")
    if float(np.median(errors)) > 1.0:
        warnings.append("Median reprojection error is above 1.0 px.")

    result: Dict[str, Any] = {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "run_dir": str(run_dir),
        "runtime_flip": runtime_flip,
        "capture_camera_params": [
            float(camera_matrix[0, 0]),
            float(camera_matrix[1, 1]),
            float(camera_matrix[0, 2]),
            float(camera_matrix[1, 2]),
        ],
        "runtime_image_size": {"width": int(runtime_width), "height": int(runtime_height)},
        "valid_frames": int(valid_frames),
        "observations": int(len(observations)),
        "rvec_target_to_camera": vector_to_list(rvec),
        "tvec_target_to_camera_m": vector_to_list(tvec),
        "rotation_target_to_camera": matrix_to_list(rotation),
        "euler_repo_convention": {
            "roll_rad": float(roll),
            "pitch_rad": float(pitch),
            "yaw_rad": float(yaw),
            "roll_deg": float(math.degrees(roll)),
            "pitch_deg": float(math.degrees(pitch)),
            "yaw_deg": float(math.degrees(yaw)),
        },
        "camera_params_real": new_camera,
        "camera_params_real_note": "Patch intrinsics target the runtime image size, not the capture resolution.",
        "realsense_tf_real": new_tf,
        "preserved_translation_xyz_m": new_tf[:3],
        "reprojection_error_px": {
            "median": float(np.median(errors)),
            "mean": float(np.mean(errors)),
            "max": float(np.max(errors)),
        },
        "frame_stats": frame_stats,
        "warnings": warnings,
    }

    write_yaml(run_dir / "calibration_result.yaml", result)
    write_reprojection_csv(run_dir / "reprojection_errors.csv", observations, projected, errors)
    write_validation_report(run_dir / "validation_report.md", result)
    write_constants_patch(run_dir / "constants_patch.txt", new_camera, new_tf, result)
    return result


def write_reprojection_csv(
    path: Path,
    observations: Sequence[Observation],
    projected: np.ndarray,
    errors: np.ndarray,
) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["image", "charuco_id", "observed_u", "observed_v", "projected_u", "projected_v", "error_px"])
        for obs, proj, error in zip(observations, projected, errors):
            writer.writerow(
                [
                    obs.image_name,
                    obs.charuco_id,
                    f"{float(obs.image_point[0]):.6f}",
                    f"{float(obs.image_point[1]):.6f}",
                    f"{float(proj[0]):.6f}",
                    f"{float(proj[1]):.6f}",
                    f"{float(error):.6f}",
                ]
            )


def write_validation_report(path: Path, result: Dict[str, Any]) -> None:
    euler = result["euler_repo_convention"]
    err = result["reprojection_error_px"]
    lines = [
        "# RealSense Mount Calibration Report",
        "",
        f"Run: `{result['run_dir']}`",
        f"Runtime flip: `{result['runtime_flip']}`",
        f"Valid frames: {result['valid_frames']}",
        f"ChArUco observations: {result['observations']}",
        "",
        "## Result",
        "",
        f"- Roll: {euler['roll_rad']:+.9f} rad ({euler['roll_deg']:+.6f} deg)",
        f"- Pitch: {euler['pitch_rad']:+.9f} rad ({euler['pitch_deg']:+.6f} deg)",
        f"- Yaw: {euler['yaw_rad']:+.9f} rad ({euler['yaw_deg']:+.6f} deg)",
        f"- Preserved translation xyz: {result['preserved_translation_xyz_m']}",
        "",
        "## Reprojection Error",
        "",
        f"- Median: {err['median']:.4f} px",
        f"- Mean: {err['mean']:.4f} px",
        f"- Max: {err['max']:.4f} px",
        "",
        "## Files",
        "",
        "- `calibration_result.yaml`",
        "- `constants_patch.txt`",
        "- `reprojection_errors.csv`",
        "- `detections/`",
    ]
    if result.get("warnings"):
        lines.extend(["", "## Warnings", ""])
        lines.extend([f"- {warning}" for warning in result["warnings"]])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def cmd_solve(args: argparse.Namespace) -> int:
    run_dir = resolve_run_dir(args.run_dir)
    result = solve_run(
        run_dir,
        int(args.min_corners),
        Path(args.constants_path).expanduser().resolve(),
        Path(args.tunables_path).expanduser().resolve(),
        int(args.runtime_width),
        int(args.runtime_height),
    )
    euler = result["euler_repo_convention"]
    err = result["reprojection_error_px"]
    print(f"Solved calibration for {run_dir}")
    print(f"  roll  {euler['roll_rad']:+.9f} rad ({euler['roll_deg']:+.6f} deg)")
    print(f"  pitch {euler['pitch_rad']:+.9f} rad ({euler['pitch_deg']:+.6f} deg)")
    print(f"  yaw   {euler['yaw_rad']:+.9f} rad ({euler['yaw_deg']:+.6f} deg)")
    print(f"  reprojection median/mean/max: {err['median']:.3f} / {err['mean']:.3f} / {err['max']:.3f} px")
    for warning in result.get("warnings", []):
        print(f"  warning: {warning}")
    print(f"Patch: {run_dir / 'constants_patch.txt'}")
    return 0


def cmd_validate(args: argparse.Namespace) -> int:
    run_dir = resolve_run_dir(args.run_dir)
    result = solve_run(
        run_dir,
        int(args.min_corners),
        Path(args.constants_path).expanduser().resolve(),
        Path(args.tunables_path).expanduser().resolve(),
        int(args.runtime_width),
        int(args.runtime_height),
    )
    print(f"Validation report refreshed: {run_dir / 'validation_report.md'}")
    for warning in result.get("warnings", []):
        print(f"  warning: {warning}")
    return 0


def cmd_wizard(args: argparse.Namespace) -> int:
    print("RealSense mount calibration wizard")
    print("This will generate the print target, capture frames, solve calibration, and write a patch.")
    print("")
    check_code = cmd_check(argparse.Namespace())
    if check_code != 0:
        print("\nFix the missing dependency above before running the wizard.")
        return check_code

    make_targets()
    print("\nStep 1: Print the target")
    print(f"  Recommended floor target: {ASSETS_DIR / DEFAULT_SPEC['pages']['a2_large2x']['file']}")
    print(f"  Small reference target:   {ASSETS_DIR / DEFAULT_SPEC['pages']['letter']['file']}")
    print(f"  Small reference target:   {ASSETS_DIR / DEFAULT_SPEC['pages']['a4']['file']}")
    print("For the low car camera, use the A2 2x target if possible.")
    print("Print at Actual size / 100% / no fit to page. Do not scale.")
    print("On the A2 2x target, the base 100 mm scale bar should measure about 200 mm.")
    input("Press Enter after the target is printed and taped flat to stiff backing...")

    measured_mm = ask_float("Measure the printed 100 mm scale bar. Enter measured length in mm", 100.0)
    print_scale = measured_mm / 100.0
    if abs(print_scale - 1.0) > 0.01:
        print(f"Print scale correction will be {print_scale:.5f}.")

    print("\nStep 2: Place the target")
    print("  Put it flat on the ground in front of the car.")
    print("  Aim +X CAR FORWARD in the car's forward direction.")
    print("  Aim +Y CAR LEFT toward the car's left side.")
    print("  Align the target centerline to the car centerline.")
    print("  Keep the car and target fixed until capture is finished.")
    input("Press Enter when the target is placed...")

    frames = ask_int("How many valid frames to capture", int(args.frames))
    capture_args = argparse.Namespace(
        run_dir=args.run_dir,
        frames=frames,
        runtime_flip=args.runtime_flip,
        print_scale_mm=measured_mm,
        min_corners=args.min_corners,
        auto=args.auto,
        interval=args.interval,
        no_window=args.no_window,
        warmup=args.warmup,
        max_seconds=args.max_seconds,
        color_width=args.color_width,
        color_height=args.color_height,
        color_fps=args.color_fps,
    )
    run_dir = capture_realsense(capture_args)
    print("\nStep 3: Solve")
    result = solve_run(
        run_dir,
        int(args.min_corners),
        Path(args.constants_path).expanduser().resolve(),
        Path(args.tunables_path).expanduser().resolve(),
        int(args.runtime_width),
        int(args.runtime_height),
    )
    euler = result["euler_repo_convention"]
    print("\nDone.")
    print(f"  roll  {euler['roll_rad']:+.9f} rad ({euler['roll_deg']:+.6f} deg)")
    print(f"  pitch {euler['pitch_rad']:+.9f} rad ({euler['pitch_deg']:+.6f} deg)")
    print(f"  yaw   {euler['yaw_rad']:+.9f} rad ({euler['yaw_deg']:+.6f} deg)")
    print(f"  patch {run_dir / 'constants_patch.txt'}")
    print(f"  report {run_dir / 'validation_report.md'}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Guided RealSense D435i mount calibration using a printed ChArUco target."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    check = sub.add_parser("check", help="Check Python/OpenCV/RealSense dependencies.")
    check.set_defaults(func=cmd_check)

    target = sub.add_parser("make-target", help="Generate print-ready calibration target assets.")
    target.set_defaults(func=cmd_make_target)

    capture = sub.add_parser("capture", help="Capture a calibration dataset from a RealSense camera.")
    add_capture_args(capture)
    capture.set_defaults(func=cmd_capture)

    solve = sub.add_parser("solve", help="Solve calibration from a captured run directory.")
    solve.add_argument("run_dir", nargs="?", help="Run directory. Defaults to the latest run.")
    solve.add_argument("--min-corners", type=int, default=MIN_CHARUCO_CORNERS)
    solve.add_argument("--constants-path", default=str(DEFAULT_CONSTANTS_PATH))
    solve.add_argument("--tunables-path", default=str(DEFAULT_TUNABLES_PATH))
    solve.add_argument("--runtime-width", type=int, default=640)
    solve.add_argument("--runtime-height", type=int, default=480)
    solve.set_defaults(func=cmd_solve)

    validate = sub.add_parser("validate", help="Refresh validation outputs for a run directory.")
    validate.add_argument("run_dir", nargs="?", help="Run directory. Defaults to the latest run.")
    validate.add_argument("--min-corners", type=int, default=MIN_CHARUCO_CORNERS)
    validate.add_argument("--constants-path", default=str(DEFAULT_CONSTANTS_PATH))
    validate.add_argument("--tunables-path", default=str(DEFAULT_TUNABLES_PATH))
    validate.add_argument("--runtime-width", type=int, default=640)
    validate.add_argument("--runtime-height", type=int, default=480)
    validate.set_defaults(func=cmd_validate)

    wizard = sub.add_parser("wizard", help="Run the guided end-to-end calibration flow.")
    add_capture_args(wizard)
    wizard.add_argument("--constants-path", default=str(DEFAULT_CONSTANTS_PATH))
    wizard.add_argument("--tunables-path", default=str(DEFAULT_TUNABLES_PATH))
    wizard.add_argument("--runtime-width", type=int, default=640)
    wizard.add_argument("--runtime-height", type=int, default=480)
    wizard.set_defaults(func=cmd_wizard)

    return parser


def add_capture_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--run-dir", help="Output run directory. Defaults to runs/YYYYMMDD_HHMMSS.")
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAME_COUNT)
    parser.add_argument("--runtime-flip", choices=["rotate180", "none"], default="rotate180")
    parser.add_argument("--print-scale-mm", type=float, default=100.0, help="Measured length of the 100 mm bar.")
    parser.add_argument("--min-corners", type=int, default=MIN_CHARUCO_CORNERS)
    parser.add_argument("--auto", action="store_true", help="Auto-save detected frames instead of using SPACE.")
    parser.add_argument("--interval", type=float, default=0.4, help="Seconds between auto-saved frames.")
    parser.add_argument("--no-window", action="store_true", help="Do not open an OpenCV preview window.")
    parser.add_argument("--warmup", type=int, default=30, help="Frames to discard after stream start.")
    parser.add_argument("--max-seconds", type=float, default=0.0, help="Stop capture after this many seconds.")
    parser.add_argument("--color-width", type=int, default=640, help="RealSense color stream width.")
    parser.add_argument("--color-height", type=int, default=480, help="RealSense color stream height.")
    parser.add_argument("--color-fps", type=int, default=30, help="RealSense color stream FPS.")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
