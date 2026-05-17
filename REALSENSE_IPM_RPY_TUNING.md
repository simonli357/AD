# RealSense IPM RPY Tuning Procedure

This procedure tunes `realsense_tf_real_lane` for lane IPM accuracy using measured
lane/ground geometry. Use it for final validation after the ChArUco and wall
calibration runs.

## 1. Lock The Runtime Convention

Use the same runtime settings as finals. If running:

```bash
roslaunch control controller.launch sign:=false show:=true depth:=false flip:=false
```

then `CAMERA_PARAMS_REAL` must be the raw, unflipped RealSense intrinsics. If
runtime uses `flip:=true`, `cx` and `cy` must be converted for the flipped image:

```text
cx_flipped = width - 1 - cx_raw
cy_flipped = height - 1 - cy_raw
```

Tune these values in:

```yaml
real:
  realsense_tf_real_lane: [x, y, z, roll, pitch, yaw]
```

Angles are radians.

## 2. Start From A Baseline

Start from the latest board/wall calibration or the best current lane-test RPY.
For the finals lane grid, use this order:

```text
pitch from stopline distances -> yaw -> roll -> pitch/yaw/roll recheck
```

This assumes the RealSense intrinsics, camera height `z`, camera forward offset
`x`, vehicle-frame reference, and physical car alignment are trusted. Under those
assumptions, tuning pitch first from the measured stopline distances is the
right first step. Do not force yaw from lane-line angle while the stopline
forward distances are still off by several centimeters.

Keep `near_m` and `far_m` fixed during calibration unless the desired controller
ROI has changed. They define the output IPM crop, not the physical camera pose.

## 3. Prepare The Finals Scene

Use the finals lane grid:

```text
lane-line spacing:        0.35 m
lane length:              1.60 m
stopline spacing:         0.20 m
car lateral placement:    car centerline aligned with lane center
car heading placement:    car heading parallel to the lanes
```

Use the center of each tape/paint line as the measured line position. With the
car centered between the two lane lines, the expected lane-line positions are:

```text
left lane line:   +0.175 m
right lane line:  -0.175 m
lane center:       0.000 m
lane width:        0.350 m
```

Use the horizontal stoplines as measured forward-distance references. If their
absolute distance from the vehicle-frame origin is known, record each stopline as
`D0`, `D1`, `D2`, etc. If the absolute origin is uncertain, the spacing is still
useful:

```text
forward_spacing_between_adjacent_stoplines = 0.20 m
```

Do not use tall objects. IPM assumes all measured points lie on the ground plane.
Avoid markers exactly at `near_m` or `far_m`; edge pixels are sensitive to FOV
and cropping. With the current `near_m = 0.48` and `far_m = 1.5`, the most useful
stoplines are the ones clearly inside that range.

## 4. Tune Pitch From Stopline Distances

Tune pitch first using all visible stoplines. Keep `near_m` and `far_m` fixed,
then measure stoplines near the bottom, middle, and top of the IPM. Prefer
stoplines clearly inside the range, for example near:

```text
0.60 m, 0.80 m, 1.00 m, 1.20 m, 1.40 m
```

Use the center of the lane for this check. If absolute stopline distances are
known, compare each measured IPM forward distance to the real distance. This is
the best pitch signal when `z`, `x`, intrinsics, and vehicle reference are known.
If only spacing is trusted, compare adjacent stopline spacing to `0.20 m`.

Pitch objective:

```text
minimize sum of forward distance errors across all visible stoplines
```

Equivalently, find the pitch value that makes each stopline's measured IPM
forward coordinate match its real-world forward coordinate.

Interpret the first check this way:

```text
absolute stopline distances match -> pitch is close enough to tune yaw
spacing is close to 0.20 m everywhere -> pitch/scale is internally consistent
spacing grows or shrinks with distance -> pitch issue, assuming z/intrinsics are correct
all stoplines shifted by about the same amount -> vehicle origin, camera x, or near_m reference issue
top/bottom edge only is wrong -> ignore edge first; use interior stoplines
```

Do not tune pitch only to make the `near_m` and `far_m` boundaries match two
measurements. Use the actual measured distances of all visible stoplines and
minimize the total error across the whole 1.6 m grid. Boundary lines can be used,
but interior lines should carry more weight because the top and bottom of the IPM
are more sensitive to FOV and cropping.

## 5. Use IPM Images, Not Screenshots

Do not measure from screenshots. Screenshots can include title bars, scaling, and
window cropping. Save or inspect the actual IPM image matrix.

Current lane IPM coordinates are:

```text
forward_m = near_m + (IPM_HEIGHT - row_px) / resolution
left_m    = (IPM_WIDTH - col_px) / resolution - width_m / 2
```

With the current `src/perception/config/lane_params.yaml`:

```yaml
resolution: 300
width_m: 0.8
near_m: 0.48
far_m: 1.5
```

the processed IPM size is approximately:

```text
IPM_WIDTH  = 0.8 * 300 = 240 px
IPM_HEIGHT = (1.5 - 0.48) * 300 = 306 px
```

## 6. Tune Yaw

Use images where the car is centered and parallel to the 35 cm lane. Before using
lane angle as a yaw measurement, verify that adjacent stoplines are roughly
20 cm apart in IPM. If they are not, fix pitch, camera height, intrinsics, or the
flip convention first.

Fit a line to the left lane and right lane in IPM. Measure their angles relative
to vertical.

Yaw objective:

```text
average_lane_angle = (left_angle + right_angle) / 2
target: average_lane_angle ~= 0 deg
```

Also fit the horizontal stoplines. Because the stoplines are physically
perpendicular to the lanes, they should be horizontal in IPM. Do not use this
alone as proof of yaw; use it together with the lane-line angle.

Current-grid yaw check:

```text
lane lines vertical
stoplines horizontal
same result near, middle, and far
```

Try:

```text
yaw0 - 0.010 rad
yaw0 - 0.005 rad
yaw0
yaw0 + 0.005 rad
yaw0 + 0.010 rad
```

Then refine with `0.001` to `0.002 rad` steps.

Check the lane angle in near, middle, and far bands. A yaw error usually appears
as a consistent rotation of both lane lines. A pitch/height problem usually shows
up as forward-distance error, changing lane width with distance, or different
near-vs-far behavior. If the near and far bands disagree, do not tune yaw from a
single global lane angle; return to the pitch check first.

If pitch is still uncertain, do a small coupled sweep around the baseline:

```text
yaw:   yaw0   +/- 0.005 rad, then +/- 0.002 rad
pitch: pitch0 +/- 0.005 rad, then +/- 0.002 rad
```

Choose the pair that minimizes both:

```text
lane verticality error
centerline forward marker error
```

Then tune roll and repeat the yaw check.

Horizontal stoplines check yaw only in the IPM image. Do not use raw camera-image
horizontality for yaw; perspective makes that ambiguous.

Use this interpretation:

```text
lane lines lean and stop line tilts consistently -> yaw error
stopline tilts but lane yaw is mostly correct -> roll/left-right asymmetry
stopline spacing is not 0.20 m -> pitch/height/scale issue
```

Success criteria:

```text
abs(average_lane_angle) < 0.5 deg
preferably < 0.3 deg
both lane lines are vertical in IPM
```

## 7. Tune Pitch

Use the stoplines. For each detected stopline, measure its forward position in
IPM at the lane center.

If absolute stopline distances are known:

```text
forward_error_i = measured_forward_i - actual_forward_i
```

If only the 20 cm spacing is trusted:

```text
spacing_error_i = (measured_forward_i+1 - measured_forward_i) - 0.20
```

Pitch objective:

```text
minimize forward_error across all stoplines, if absolute distances are known
minimize spacing_error across adjacent stoplines, always
```

Interpretation:

```text
20 cm spacing grows/shrinks with distance -> pitch/height/scale issue
all stoplines shifted by similar amount -> vehicle-origin or near_m reference issue
lane width wrong everywhere too -> intrinsics/height/RPY coupling issue
```

Do not tune `near_m` to hide bad pitch unless the physical distance reference is
known to be wrong. `near_m` defines the output crop start; it is not the camera
orientation.

Success criteria:

```text
forward MAE < 2 cm
forward max error < 4 cm
errors do not grow strongly with distance
```

## 8. Tune Roll

Use the horizontal stoplines and the left/right lane-line intersections.

Roll objective:

```text
each stopline should have the same forward coordinate on the left and right
each stopline should be horizontal after yaw is close
lane width should stay 0.35 m from near to far
```

Metric:

```text
roll_error_i = forward_ipm_at_left_lane_i - forward_ipm_at_right_lane_i
```

Tune roll until this is near zero across near, mid, and far distances.

Success criteria:

```text
left-right same-stopline mismatch < 2 cm
stopline angle < 0.5 deg from horizontal
lane width stays 0.35 m from bottom to top
lane center stays near 0.00 m from bottom to top
```

## 9. Recheck Coupling

Yaw, pitch, and roll are coupled. After pitch and roll tuning, repeat:

```text
yaw check -> pitch check -> roll check
```

Usually one extra pass is enough.

## 10. Final Acceptance Criteria

Use at least three different clean frames or car positions.

Accept the final RPY only if:

```text
straight-lane yaw error:      < 0.5 deg, preferably < 0.3 deg
left/right lane angle diff:   < 0.5 deg
forward marker MAE:           < 2 cm
forward marker max error:     < 4 cm
stopline spacing error:       < 2 cm, preferably < 1 cm
left-right marker mismatch:   < 2 cm
lane width mean error:        < 2 cm from 0.35 m
lane width std over distance: < 1.5-2 cm
lane center error:            < 2 cm from 0.00 m
good_left/good_right:         both true on clean straight-lane frames
```

Black regions in `ipm_color` show camera FOV coverage, not lane detection
quality. Use them as a diagnostic only. Final RPY should be chosen from measured
lane-line angle, marker position error, and left/right symmetry.
