#!/usr/bin/env python3
"""
Estimate RealSense yaw from depth observations of a flat wall.

Setup assumption:
  - The car centerline is perpendicular to a flat wall.
  - The wall fills the selected depth ROI.
  - The RealSense is mounted normally.

The default output convention matches the repo's runtime camera path, where
frames are rotated 180 degrees before perception uses them.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


SCRIPT_DIR = Path(__file__).resolve().parent
RUNS_DIR = SCRIPT_DIR / "runs"
TARGET_YAW_STD_WARN_DEG = 0.15
TARGET_RESIDUAL_WARN_M = 0.015


@dataclass(frozen=True)
class DepthIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass
class FrameYawResult:
    frame_index: int
    points: int
    inliers: int
    inlier_ratio: float
    normal: np.ndarray
    plane_d: float
    yaw_rad: float
    yaw_deg: float
    residual_median_m: float
    residual_p95_m: float
    left_depth_m: Optional[float]
    right_depth_m: Optional[float]
    right_minus_left_depth_m: Optional[float]
    left_right_yaw_rad: Optional[float]
    left_right_yaw_deg: Optional[float]


def now_stamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def require_yaml() -> Any:
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install it with: python3 -m pip install pyyaml")
    return yaml


def write_yaml(path: Path, data: Dict[str, Any]) -> None:
    y = require_yaml()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        y.safe_dump(data, fh, sort_keys=False)


def create_run_dir(path_text: Optional[str]) -> Path:
    if path_text:
        run_dir = Path(path_text).expanduser().resolve()
    else:
        run_dir = RUNS_DIR / f"{now_stamp()}_depth_wall_yaw"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def intrinsics_from_rs(intrinsics: Any) -> DepthIntrinsics:
    return DepthIntrinsics(
        width=int(intrinsics.width),
        height=int(intrinsics.height),
        fx=float(intrinsics.fx),
        fy=float(intrinsics.fy),
        ppx=float(intrinsics.ppx),
        ppy=float(intrinsics.ppy),
    )


def intrinsics_to_dict(intrinsics: DepthIntrinsics) -> Dict[str, float]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
    }


def adjust_intrinsics_for_rotate180(intrinsics: DepthIntrinsics) -> DepthIntrinsics:
    return DepthIntrinsics(
        width=intrinsics.width,
        height=intrinsics.height,
        fx=intrinsics.fx,
        fy=intrinsics.fy,
        ppx=float(intrinsics.width - 1) - intrinsics.ppx,
        ppy=float(intrinsics.height - 1) - intrinsics.ppy,
    )


def apply_runtime_flip_depth(depth_m: np.ndarray, runtime_flip: str) -> np.ndarray:
    if runtime_flip == "none":
        return depth_m
    if runtime_flip == "rotate180":
        return np.ascontiguousarray(depth_m[::-1, ::-1])
    raise ValueError(f"Unsupported runtime flip: {runtime_flip}")


def normalized_roi(roi: Sequence[float], width: int, height: int) -> Tuple[int, int, int, int]:
    if len(roi) != 4:
        raise ValueError("--roi expects four normalized values: x0 y0 x1 y1")
    x0, y0, x1, y1 = [float(v) for v in roi]
    if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
        raise ValueError(f"ROI values must satisfy 0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1; got {roi}")
    px0 = max(0, min(width - 1, int(round(x0 * width))))
    px1 = max(px0 + 1, min(width, int(round(x1 * width))))
    py0 = max(0, min(height - 1, int(round(y0 * height))))
    py1 = max(py0 + 1, min(height, int(round(y1 * height))))
    return px0, py0, px1, py1


def sample_depth_points(
    depth_m: np.ndarray,
    intrinsics: DepthIntrinsics,
    roi_px: Tuple[int, int, int, int],
    stride: int,
    min_distance_m: float,
    max_distance_m: float,
    max_points: int,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, np.ndarray]:
    x0, y0, x1, y1 = roi_px
    stride = max(1, int(stride))
    us = np.arange(x0, x1, stride, dtype=np.float64)
    vs = np.arange(y0, y1, stride, dtype=np.float64)
    grid_u, grid_v = np.meshgrid(us, vs)
    u_flat = grid_u.reshape(-1)
    v_flat = grid_v.reshape(-1)
    z = depth_m[v_flat.astype(np.int32), u_flat.astype(np.int32)].astype(np.float64)
    valid = np.isfinite(z) & (z >= float(min_distance_m)) & (z <= float(max_distance_m))
    u_flat = u_flat[valid]
    v_flat = v_flat[valid]
    z = z[valid]
    if z.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 2), dtype=np.float64)

    if max_points > 0 and z.size > max_points:
        idx = rng.choice(z.size, size=int(max_points), replace=False)
        u_flat = u_flat[idx]
        v_flat = v_flat[idx]
        z = z[idx]

    x = (u_flat - intrinsics.ppx) / intrinsics.fx * z
    y = (v_flat - intrinsics.ppy) / intrinsics.fy * z
    points = np.column_stack([x, y, z]).astype(np.float64)
    pixels = np.column_stack([u_flat, v_flat]).astype(np.float64)
    return points, pixels


def sample_binned_depth_points(
    depth_m: np.ndarray,
    intrinsics: DepthIntrinsics,
    roi_px: Tuple[int, int, int, int],
    bin_cols: int,
    bin_rows: int,
    min_distance_m: float,
    max_distance_m: float,
    min_bin_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    x0, y0, x1, y1 = roi_px
    xs = np.linspace(x0, x1, int(bin_cols) + 1, dtype=np.int32)
    ys = np.linspace(y0, y1, int(bin_rows) + 1, dtype=np.int32)
    points: List[List[float]] = []
    pixels: List[List[float]] = []

    for row in range(int(bin_rows)):
        by0, by1 = int(ys[row]), int(ys[row + 1])
        if by1 <= by0:
            continue
        for col in range(int(bin_cols)):
            bx0, bx1 = int(xs[col]), int(xs[col + 1])
            if bx1 <= bx0:
                continue
            patch = depth_m[by0:by1, bx0:bx1]
            valid = np.isfinite(patch) & (patch >= float(min_distance_m)) & (patch <= float(max_distance_m))
            if int(np.count_nonzero(valid)) < int(min_bin_points):
                continue
            z = float(np.median(patch[valid]))
            valid_y, valid_x = np.nonzero(valid)
            u = float(bx0 + np.median(valid_x))
            v = float(by0 + np.median(valid_y))
            x = (u - intrinsics.ppx) / intrinsics.fx * z
            y = (v - intrinsics.ppy) / intrinsics.fy * z
            points.append([x, y, z])
            pixels.append([u, v])

    if not points:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    return np.asarray(points, dtype=np.float64), np.asarray(pixels, dtype=np.float64)


def normalize_plane(normal: np.ndarray, d: float) -> Tuple[np.ndarray, float]:
    normal = np.asarray(normal, dtype=np.float64).reshape(3)
    scale = np.linalg.norm(normal)
    if scale <= 1e-12:
        raise ValueError("Plane normal has near-zero length")
    normal = normal / scale
    d = float(d) / scale
    if normal[2] < 0.0:
        normal = -normal
        d = -d
    return normal, d


def fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    if points.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit a plane")
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1, :]
    d = -float(np.dot(normal, centroid))
    return normalize_plane(normal, d)


def plane_residuals(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    return np.abs(points @ normal.reshape(3) + float(d))


def fit_plane_ransac(
    points: np.ndarray,
    threshold_m: float,
    iterations: int,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, float, np.ndarray]:
    if points.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit a plane")

    best_inliers: Optional[np.ndarray] = None
    best_count = -1
    best_residual = float("inf")
    n_points = points.shape[0]
    threshold_m = float(threshold_m)

    for _ in range(max(1, int(iterations))):
        idx = rng.choice(n_points, size=3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm <= 1e-12:
            continue
        d = -float(np.dot(normal, p0))
        normal, d = normalize_plane(normal, d)
        residuals = plane_residuals(points, normal, d)
        inliers = residuals <= threshold_m
        count = int(np.count_nonzero(inliers))
        median_residual = float(np.median(residuals[inliers])) if count else float("inf")
        if count > best_count or (count == best_count and median_residual < best_residual):
            best_inliers = inliers
            best_count = count
            best_residual = median_residual

    if best_inliers is None or best_count < 3:
        normal, d = fit_plane_svd(points)
        return normal, d, np.ones(points.shape[0], dtype=bool)

    normal, d = fit_plane_svd(points[best_inliers])
    residuals = plane_residuals(points, normal, d)
    inliers = residuals <= threshold_m
    if int(np.count_nonzero(inliers)) >= 3:
        normal, d = fit_plane_svd(points[inliers])
    return normal, d, inliers


def yaw_from_wall_normal(normal: np.ndarray) -> float:
    normal = np.asarray(normal, dtype=np.float64).reshape(3)
    if normal[2] < 0.0:
        normal = -normal
    return float(math.atan2(float(normal[0]), float(normal[2])))


def left_right_yaw_from_points(
    points: np.ndarray,
    pixels: np.ndarray,
    roi_px: Tuple[int, int, int, int],
    edge_band: float,
    min_edge_points: int = 50,
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    x0, _, x1, _ = roi_px
    edge_width = max(1.0, float(edge_band) * float(x1 - x0))
    left = pixels[:, 0] <= (float(x0) + edge_width)
    right = pixels[:, 0] >= (float(x1) - edge_width)
    if np.count_nonzero(left) < min_edge_points or np.count_nonzero(right) < min_edge_points:
        return None, None, None, None

    left_depth = float(np.median(points[left, 2]))
    right_depth = float(np.median(points[right, 2]))
    left_x = float(np.median(points[left, 0]))
    right_x = float(np.median(points[right, 0]))
    dx = right_x - left_x
    if abs(dx) <= 1e-9:
        return left_depth, right_depth, right_depth - left_depth, None

    dz_dx = (right_depth - left_depth) / dx
    yaw = float(math.atan2(-dz_dx, 1.0))
    return left_depth, right_depth, right_depth - left_depth, yaw


def solve_frame(
    frame_index: int,
    depth_m: np.ndarray,
    intrinsics: DepthIntrinsics,
    args: argparse.Namespace,
    rng: np.random.Generator,
) -> Optional[FrameYawResult]:
    depth_m = apply_runtime_flip_depth(depth_m, args.runtime_flip)
    active_intrinsics = intrinsics
    if args.runtime_flip == "rotate180":
        active_intrinsics = adjust_intrinsics_for_rotate180(intrinsics)

    roi_px = normalized_roi(args.roi, active_intrinsics.width, active_intrinsics.height)
    if int(args.bin_cols) > 0 and int(args.bin_rows) > 0:
        points, pixels = sample_binned_depth_points(
            depth_m,
            active_intrinsics,
            roi_px,
            int(args.bin_cols),
            int(args.bin_rows),
            float(args.min_distance),
            float(args.max_distance),
            int(args.min_bin_points),
        )
    else:
        points, pixels = sample_depth_points(
            depth_m,
            active_intrinsics,
            roi_px,
            int(args.stride),
            float(args.min_distance),
            float(args.max_distance),
            int(args.max_points),
            rng,
        )
    if points.shape[0] < int(args.min_points):
        return None

    normal, d, inlier_mask = fit_plane_ransac(
        points,
        float(args.ransac_threshold_m),
        int(args.ransac_iterations),
        rng,
    )
    residuals = plane_residuals(points[inlier_mask], normal, d)
    yaw = yaw_from_wall_normal(normal)
    left_depth, right_depth, right_minus_left, lr_yaw = left_right_yaw_from_points(
        points,
        pixels,
        roi_px,
        float(args.edge_band),
    )
    return FrameYawResult(
        frame_index=int(frame_index),
        points=int(points.shape[0]),
        inliers=int(np.count_nonzero(inlier_mask)),
        inlier_ratio=float(np.count_nonzero(inlier_mask)) / float(points.shape[0]),
        normal=normal,
        plane_d=float(d),
        yaw_rad=float(yaw),
        yaw_deg=float(math.degrees(yaw)),
        residual_median_m=float(np.median(residuals)) if residuals.size else float("nan"),
        residual_p95_m=float(np.percentile(residuals, 95)) if residuals.size else float("nan"),
        left_depth_m=left_depth,
        right_depth_m=right_depth,
        right_minus_left_depth_m=right_minus_left,
        left_right_yaw_rad=lr_yaw,
        left_right_yaw_deg=math.degrees(lr_yaw) if lr_yaw is not None else None,
    )


def summarize_results(results: Sequence[FrameYawResult], args: argparse.Namespace) -> Dict[str, Any]:
    if not results:
        raise RuntimeError("No valid depth frames. Check wall visibility, ROI, and min/max distance filters.")

    weights = np.array([max(1, r.inliers) for r in results], dtype=np.float64)
    normals = np.array([r.normal for r in results], dtype=np.float64)
    weighted_normal = np.sum(normals * weights[:, None], axis=0)
    weighted_normal, _ = normalize_plane(weighted_normal, 0.0)
    weighted_yaw_rad = yaw_from_wall_normal(weighted_normal)
    yaws_rad = np.array([r.yaw_rad for r in results], dtype=np.float64)
    lr_yaws_rad = np.array([r.left_right_yaw_rad for r in results if r.left_right_yaw_rad is not None], dtype=np.float64)
    residual_medians = np.array([r.residual_median_m for r in results], dtype=np.float64)
    residual_p95s = np.array([r.residual_p95_m for r in results], dtype=np.float64)

    warnings: List[str] = []
    yaw_std_deg = float(np.std(np.degrees(yaws_rad)))
    residual_p95_median = float(np.median(residual_p95s))
    if len(results) > 1 and yaw_std_deg > TARGET_YAW_STD_WARN_DEG:
        warnings.append(f"Frame-to-frame yaw stddev is {yaw_std_deg:.3f} deg; target or depth may be unstable.")
    if residual_p95_median > TARGET_RESIDUAL_WARN_M:
        warnings.append(f"Median p95 plane residual is {residual_p95_median * 1000.0:.1f} mm; wall ROI may include clutter or depth noise.")
    if args.runtime_flip != "rotate180":
        warnings.append("runtime_flip is not rotate180; reported yaw may not match the default perception camera convention.")

    left_depths = [r.left_depth_m for r in results if r.left_depth_m is not None]
    right_depths = [r.right_depth_m for r in results if r.right_depth_m is not None]
    right_minus_left = [r.right_minus_left_depth_m for r in results if r.right_minus_left_depth_m is not None]

    return {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "runtime_flip": args.runtime_flip,
        "roi_normalized": [float(v) for v in args.roi],
        "frames_requested": int(args.frames),
        "valid_frames": int(len(results)),
        "temporal_median": bool(args.temporal_median),
        "wall_yaw_repo_convention": {
            "rad": float(weighted_yaw_rad),
            "deg": float(math.degrees(weighted_yaw_rad)),
            "note": "Positive means the camera points left relative to the car centerline; with rotate180 this matches realsense_tf_real yaw.",
        },
        "wall_yaw_frame_median": {
            "rad": float(np.median(yaws_rad)),
            "deg": float(math.degrees(float(np.median(yaws_rad)))),
            "std_deg": yaw_std_deg,
            "min_deg": float(np.min(np.degrees(yaws_rad))),
            "max_deg": float(np.max(np.degrees(yaws_rad))),
        },
        "left_right_depth_check": {
            "left_depth_m_median": float(np.median(left_depths)) if left_depths else None,
            "right_depth_m_median": float(np.median(right_depths)) if right_depths else None,
            "right_minus_left_depth_m_median": float(np.median(right_minus_left)) if right_minus_left else None,
            "yaw_from_left_right_rad_median": float(np.median(lr_yaws_rad)) if lr_yaws_rad.size else None,
            "yaw_from_left_right_deg_median": float(math.degrees(float(np.median(lr_yaws_rad)))) if lr_yaws_rad.size else None,
            "note": "For positive yaw, the right side of the runtime image should usually be closer than the left side.",
        },
        "plane": {
            "normal_weighted_mean": [float(v) for v in weighted_normal],
            "distance_m_median": float(np.median([abs(r.plane_d) for r in results])),
            "residual_median_m_median": float(np.median(residual_medians)),
            "residual_p95_m_median": residual_p95_median,
            "inlier_ratio_median": float(np.median([r.inlier_ratio for r in results])),
        },
        "filters": {
            "min_distance_m": float(args.min_distance),
            "max_distance_m": float(args.max_distance),
            "stride": int(args.stride),
            "max_points": int(args.max_points),
            "bin_cols": int(args.bin_cols),
            "bin_rows": int(args.bin_rows),
            "min_bin_points": int(args.min_bin_points),
            "ransac_threshold_m": float(args.ransac_threshold_m),
            "ransac_iterations": int(args.ransac_iterations),
        },
        "warnings": warnings,
    }


def frame_result_to_row(result: FrameYawResult) -> List[Any]:
    return [
        result.frame_index,
        result.points,
        result.inliers,
        f"{result.inlier_ratio:.6f}",
        f"{result.yaw_rad:.12f}",
        f"{result.yaw_deg:.9f}",
        f"{result.normal[0]:.12f}",
        f"{result.normal[1]:.12f}",
        f"{result.normal[2]:.12f}",
        f"{result.plane_d:.12f}",
        f"{result.residual_median_m:.9f}",
        f"{result.residual_p95_m:.9f}",
        "" if result.left_depth_m is None else f"{result.left_depth_m:.9f}",
        "" if result.right_depth_m is None else f"{result.right_depth_m:.9f}",
        "" if result.right_minus_left_depth_m is None else f"{result.right_minus_left_depth_m:.9f}",
        "" if result.left_right_yaw_rad is None else f"{result.left_right_yaw_rad:.12f}",
        "" if result.left_right_yaw_deg is None else f"{result.left_right_yaw_deg:.9f}",
    ]


def write_frame_csv(path: Path, results: Sequence[FrameYawResult]) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "frame",
                "points",
                "inliers",
                "inlier_ratio",
                "yaw_rad",
                "yaw_deg",
                "normal_x",
                "normal_y",
                "normal_z",
                "plane_d",
                "residual_median_m",
                "residual_p95_m",
                "left_depth_m",
                "right_depth_m",
                "right_minus_left_depth_m",
                "left_right_yaw_rad",
                "left_right_yaw_deg",
            ]
        )
        for result in results:
            writer.writerow(frame_result_to_row(result))


def write_report(path: Path, result: Dict[str, Any]) -> None:
    yaw = result["wall_yaw_repo_convention"]
    yaw_frames = result["wall_yaw_frame_median"]
    lr = result["left_right_depth_check"]
    plane = result["plane"]
    lines = [
        "# Depth Wall Yaw Report",
        "",
        f"Runtime flip: `{result['runtime_flip']}`",
        f"Depth frames used: {result.get('depth_frames_used', result['valid_frames'])} / {result['frames_requested']}",
        f"Plane fits: {result['valid_frames']}",
        f"Temporal median: `{result.get('temporal_median', False)}`",
        "",
        "## Result",
        "",
        f"- Yaw: {yaw['rad']:+.9f} rad ({yaw['deg']:+.6f} deg)",
        f"- Frame median yaw: {yaw_frames['rad']:+.9f} rad ({yaw_frames['deg']:+.6f} deg)",
        f"- Frame yaw stddev: {yaw_frames['std_deg']:.6f} deg",
        "",
        "## Left/Right Check",
        "",
        f"- Left depth median: {format_optional(lr['left_depth_m_median'], 'm')}",
        f"- Right depth median: {format_optional(lr['right_depth_m_median'], 'm')}",
        f"- Right minus left depth: {format_optional(lr['right_minus_left_depth_m_median'], 'm')}",
        f"- Left/right yaw median: {format_optional(lr['yaw_from_left_right_deg_median'], 'deg')}",
        "",
        "## Plane Fit",
        "",
        f"- Wall distance median: {plane['distance_m_median']:.4f} m",
        f"- Plane residual median: {plane['residual_median_m_median'] * 1000.0:.2f} mm",
        f"- Plane residual p95 median: {plane['residual_p95_m_median'] * 1000.0:.2f} mm",
        f"- Inlier ratio median: {plane['inlier_ratio_median']:.3f}",
        "",
        "## Files",
        "",
        "- `depth_wall_yaw_result.yaml`",
        "- `depth_wall_yaw_frames.csv`",
    ]
    if result.get("warnings"):
        lines.extend(["", "## Warnings", ""])
        lines.extend([f"- {warning}" for warning in result["warnings"]])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def format_optional(value: Optional[float], unit: str) -> str:
    if value is None:
        return "n/a"
    if unit == "m":
        return f"{float(value):+.4f} m"
    if unit == "deg":
        return f"{float(value):+.6f} deg"
    return f"{float(value):+.6f} {unit}"


def set_emitter(depth_sensor: Any, emitter: str) -> None:
    if emitter == "auto":
        return
    try:
        import pyrealsense2 as rs

        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1.0 if emitter == "on" else 0.0)
    except Exception as exc:
        print(f"Warning: could not set emitter={emitter}: {exc}")


def capture_and_solve(args: argparse.Namespace) -> Dict[str, Any]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        raise RuntimeError("pyrealsense2 is required. Install it on the machine connected to the RealSense.") from exc

    run_dir = create_run_dir(args.run_dir)
    rng = np.random.default_rng(int(args.seed))
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, int(args.width), int(args.height), rs.format.z16, int(args.fps))

    print("Starting RealSense depth stream...")
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    set_emitter(depth_sensor, str(args.emitter))
    depth_scale = float(depth_sensor.get_depth_scale())
    stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intrinsics = intrinsics_from_rs(stream.get_intrinsics())

    results: List[FrameYawResult] = []
    raw_depth_frames: List[np.ndarray] = []
    frame_index = 0
    started_at = time.time()
    try:
        for _ in range(int(args.warmup)):
            pipeline.wait_for_frames()
        while (len(raw_depth_frames) if args.temporal_median else len(results)) < int(args.frames):
            if args.max_seconds and time.time() - started_at >= float(args.max_seconds):
                print("Reached capture time limit.")
                break
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            depth_raw = np.asanyarray(depth_frame.get_data()).copy()
            if args.temporal_median:
                raw_depth_frames.append(depth_raw)
                print(f"Captured depth frame {len(raw_depth_frames):03d}/{args.frames}")
                continue
            depth_m = depth_raw.astype(np.float64) * depth_scale
            frame_result = solve_frame(frame_index, depth_m, intrinsics, args, rng)
            frame_index += 1
            if frame_result is None:
                if args.verbose:
                    print(f"Skipped frame {frame_index}: not enough valid wall points.")
                continue
            results.append(frame_result)
            print(
                f"Frame {len(results):03d}/{args.frames}: "
                f"yaw={frame_result.yaw_deg:+.4f} deg, "
                f"p95={frame_result.residual_p95_m * 1000.0:.1f} mm, "
                f"inliers={frame_result.inlier_ratio:.2f}"
            )
    finally:
        pipeline.stop()

    if args.temporal_median:
        if not raw_depth_frames:
            raise RuntimeError("No depth frames captured.")
        depth_m = temporal_median_depth(raw_depth_frames, depth_scale)
        frame_result = solve_frame(-1, depth_m, intrinsics, args, rng)
        if frame_result is None:
            raise RuntimeError("Temporal median depth image did not contain enough valid wall points.")
        results.append(frame_result)
        print(
            "Temporal median solve: "
            f"yaw={frame_result.yaw_deg:+.4f} deg, "
            f"p95={frame_result.residual_p95_m * 1000.0:.1f} mm, "
            f"inliers={frame_result.inlier_ratio:.2f}"
        )

    summary = summarize_results(results, args)
    summary["run_dir"] = str(run_dir)
    summary["depth_frames_used"] = int(len(raw_depth_frames) if args.temporal_median else len(results))
    summary["depth_intrinsics_raw"] = intrinsics_to_dict(intrinsics)
    summary["depth_intrinsics_runtime"] = intrinsics_to_dict(
        adjust_intrinsics_for_rotate180(intrinsics) if args.runtime_flip == "rotate180" else intrinsics
    )
    summary["depth_scale_m_per_unit"] = depth_scale
    summary["stream"] = {"width": int(args.width), "height": int(args.height), "fps": int(args.fps)}

    write_yaml(run_dir / "depth_wall_yaw_result.yaml", summary)
    write_frame_csv(run_dir / "depth_wall_yaw_frames.csv", results)
    write_report(run_dir / "depth_wall_yaw_report.md", summary)
    return summary


def temporal_median_depth(raw_depth_frames: Sequence[np.ndarray], depth_scale: float) -> np.ndarray:
    stack = np.stack(raw_depth_frames, axis=0).astype(np.float32)
    stack[stack <= 0.0] = np.nan
    with np.errstate(all="ignore"):
        median_raw = np.nanmedian(stack, axis=0)
    median_raw = np.nan_to_num(median_raw, nan=0.0)
    return median_raw.astype(np.float64) * float(depth_scale)


def cmd_check(_: argparse.Namespace) -> int:
    failed = False
    print("Depth wall yaw dependency check")
    print(f"  OK   numpy: {np.__version__}")
    print(f"  {'OK  ' if yaml is not None else 'MISS'} pyyaml: {'ok' if yaml is not None else 'missing'}")
    failed = failed or yaml is None
    try:
        import pyrealsense2 as rs  # noqa: F401

        print("  OK   pyrealsense2: ok")
    except Exception as exc:
        print(f"  MISS pyrealsense2: {type(exc).__name__}: {exc}")
        failed = True
    if failed:
        print("\nInstall missing Python packages with:")
        print("  python3 -m pip install -r src/perception/scripts/calibration/requirements-calibration.txt")
        return 1
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    result = capture_and_solve(args)
    yaw = result["wall_yaw_repo_convention"]
    lr = result["left_right_depth_check"]
    plane = result["plane"]
    print("\nDepth wall yaw complete.")
    print(f"  yaw: {yaw['rad']:+.9f} rad ({yaw['deg']:+.6f} deg)")
    print(f"  left depth median:  {format_optional(lr['left_depth_m_median'], 'm')}")
    print(f"  right depth median: {format_optional(lr['right_depth_m_median'], 'm')}")
    print(f"  right-left depth:   {format_optional(lr['right_minus_left_depth_m_median'], 'm')}")
    print(f"  residual median/p95: {plane['residual_median_m_median'] * 1000.0:.2f} / {plane['residual_p95_m_median'] * 1000.0:.2f} mm")
    for warning in result.get("warnings", []):
        print(f"  warning: {warning}")
    print(f"  report: {Path(result['run_dir']) / 'depth_wall_yaw_report.md'}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Estimate RealSense yaw from D435i depth measurements of a flat wall."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    check = sub.add_parser("check", help="Check depth yaw dependencies.")
    check.set_defaults(func=cmd_check)

    run = sub.add_parser("run", help="Capture depth frames and estimate yaw from a wall plane.")
    run.add_argument("--run-dir", help="Output run directory. Defaults to runs/YYYYMMDD_HHMMSS_depth_wall_yaw.")
    run.add_argument("--frames", type=int, default=90, help="Valid depth frames to use.")
    run.add_argument("--warmup", type=int, default=30, help="Depth frames to discard after stream start.")
    run.add_argument("--max-seconds", type=float, default=120.0, help="Stop after this many seconds.")
    run.add_argument("--width", type=int, default=848, help="RealSense depth stream width.")
    run.add_argument("--height", type=int, default=480, help="RealSense depth stream height.")
    run.add_argument("--fps", type=int, default=30, help="RealSense depth stream FPS.")
    run.add_argument("--runtime-flip", choices=["rotate180", "none"], default="rotate180")
    run.add_argument(
        "--roi",
        type=float,
        nargs=4,
        default=[0.12, 0.25, 0.88, 0.75],
        metavar=("X0", "Y0", "X1", "Y1"),
        help="Normalized ROI containing only the flat wall.",
    )
    run.add_argument("--edge-band", type=float, default=0.18, help="Fraction of ROI width used for left/right depth check.")
    run.add_argument("--min-distance", type=float, default=0.6, help="Reject depth points closer than this.")
    run.add_argument("--max-distance", type=float, default=2.2, help="Reject depth points farther than this.")
    run.add_argument("--stride", type=int, default=4, help="Pixel sampling stride in the ROI.")
    run.add_argument("--max-points", type=int, default=12000, help="Maximum sampled points per frame.")
    run.add_argument("--min-points", type=int, default=200, help="Minimum valid points required for a plane fit.")
    run.add_argument("--bin-cols", type=int, default=48, help="Median depth bins across the ROI. Use 0 to disable binning.")
    run.add_argument("--bin-rows", type=int, default=24, help="Median depth bins down the ROI. Use 0 to disable binning.")
    run.add_argument("--min-bin-points", type=int, default=8, help="Minimum valid depth pixels per median bin.")
    run.add_argument("--ransac-threshold-m", type=float, default=0.012, help="Plane inlier threshold.")
    run.add_argument("--ransac-iterations", type=int, default=160)
    run.add_argument("--emitter", choices=["auto", "on", "off"], default="on")
    run.add_argument("--seed", type=int, default=7)
    run.add_argument("--no-temporal-median", dest="temporal_median", action="store_false")
    run.add_argument("--verbose", action="store_true")
    run.set_defaults(temporal_median=True)
    run.set_defaults(func=cmd_run)
    return parser


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
