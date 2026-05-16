# RealSense Mount Calibration

Use this folder to calibrate the D435i mount angles used by the lane IPM code.
The old `yaw.py` and `roll.py` scripts are legacy sanity checks only. Use
`calibrate_realsense_mount.py` for real calibration.

## 1. Check Dependencies

From the repository root:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py check
```

Install missing Python packages with:

```bash
python3 -m pip install -r src/perception/scripts/calibration/requirements-calibration.txt
```

OpenCV must include `cv2.aruco`. If your system OpenCV does not, install an
OpenCV contrib build for the calibration environment.

## 2. Print The Target

Regenerate the target files if needed:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py make-target
```

For this car's low-mounted camera, print the larger floor target if possible:

- `assets/charuco_ground_target_a2_large_2x_300dpi.png`

For the alternate wall-mounted procedure, print:

- `assets/charuco_wall_target_a2_large_2x_300dpi.png`

It is an A2 landscape target. Print it at `Actual size / 100% / no fit to
page`. Its board is about `48 cm x 36 cm`; the base `100 mm` scale bar should
measure about `200 mm`. Enter the actual measured scale-bar length when the
wizard asks.

The smaller files are kept for close-range checks, but they are usually too
small for floor calibration with the car camera:

- `assets/charuco_ground_target_letter_300dpi.png`
- `assets/charuco_ground_target_a4_300dpi.png`
- `assets/charuco_wall_target_letter_300dpi.png`
- `assets/charuco_wall_target_a4_300dpi.png`

Printer settings:

- Print at `Actual size`, `100%`, or `no fit to page`.
- Use landscape orientation.
- Do not scale.
- After printing, measure the scale check bar. You will enter that value into
  the wizard. For the recommended large target, this should be around `200 mm`.

Tape the page flat to stiff backing. A warped target gives bad calibration.

The scale-bar correction handles uniform printer scaling. It does not solve the
visibility problem caused by tiny markers. The target must be large and sharp
enough that OpenCV can detect at least 12 ChArUco corners from the car camera.

## 3. Place The Target

Park the car on a flat surface with the camera mounted normally.

### Ground Procedure

Place the target flat on the ground in front of the car:

- `+X CAR FORWARD` points in the car's forward direction.
- `+Y CAR LEFT` points toward the car's left side.
- The target centerline is aligned with the car centerline.
- The car and target do not move during capture.

Yaw accuracy depends mostly on this physical alignment.

### Wall Procedure

Use this when the floor target is hard to keep flat or too close to the bottom
of the image. The target must be on a rigid, flat, vertical surface in front of
the car.

Place the wall target facing the camera:

- The target plane is perpendicular to the car centerline.
- `+Z CAR UP` points upward.
- `+Y CAR LEFT` points toward the car's left side.
- The target vertical centerline is aligned with the car centerline.
- The car and target do not move during capture.

The wall method can improve marker detection because the board is flatter and
more visible. It is not automatically more accurate: yaw accuracy now depends
on the wall or backing board being square to the car centerline. Use a level
and a centerline/string/laser reference if possible.

## 4. Run The Wizard

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py wizard
```

Default behavior matches `cameraNode.launch`, where RealSense frames are rotated
180 degrees before runtime perception uses them.

During capture:

- Press `SPACE` to save a frame when enough ChArUco corners are detected.
- Press `q` to finish early.
- Capture at least 15 valid frames; 40 is the default.

For automatic capture:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py wizard --auto
```

For the wall procedure:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py wizard --target-placement vertical-front --auto
```

If the checker sees too few corners at the default runtime resolution, capture
at a higher RealSense color resolution:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py capture --auto --no-window --color-width 1920 --color-height 1080
```

For high-resolution wall capture without the wizard:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py capture --target-placement vertical-front --auto --no-window --color-width 1920 --color-height 1080
```

## 5. Use The Result

Each run writes files under:

```text
src/perception/scripts/calibration/runs/YYYYMMDD_HHMMSS/
```

Important outputs:

- `constants_patch.txt`: copy `CAMERA_PARAMS_REAL` into
  `src/utils/include/utils/constants.h`, and `realsense_tf_real` into
  `src/control/config/tunable_params.yaml`.
- `calibration_result.yaml`: full numeric result.
- `validation_report.md`: concise human-readable report.
- `reprojection_errors.csv`: per-corner reprojection errors.
- `detections/`: saved images with detected markers overlaid.

The tool preserves the existing camera `x/y/z` translation and only replaces
the calibrated roll, pitch, and yaw. It also prints the effective camera
intrinsics used for the runtime-flipped image.

## 6. Depth-Only Wall Yaw Check

`depth_wall_yaw.py` estimates yaw from the D435i depth image only. Use it as an
independent yaw sanity check when the car is square to a flat wall about
`1.0-1.5 m` away.

Setup:

- The wall must be flat and perpendicular to the car centerline.
- The selected image ROI must contain only the wall, not the target paper edge,
  tape, floor, car bumper, or other objects.
- Default output uses raw RealSense depth geometry. Positive yaw means the
  camera points left, and the right side of the raw depth image should be
  closer than the left side. This is the value to compare against
  `realsense_tf_real` yaw.

Run on the machine connected to the RealSense:

```bash
python3 src/perception/scripts/calibration/depth_wall_yaw.py check
python3 src/perception/scripts/calibration/depth_wall_yaw.py run
```

Useful options:

```bash
python3 src/perception/scripts/calibration/depth_wall_yaw.py run \
  --frames 120 \
  --width 848 --height 480 \
  --min-distance 0.8 --max-distance 1.8 \
  --roi 0.25 0.30 0.75 0.70
```

Outputs are written under `runs/YYYYMMDD_HHMMSS_depth_wall_yaw/`:

- `depth_wall_yaw_report.md`: human-readable yaw and left/right depth summary.
- `depth_wall_yaw_result.yaml`: full numeric result.
- `depth_wall_yaw_frames.csv`: per-frame plane-fit details.

This method estimates yaw only. It does not replace the ChArUco workflow for
roll/pitch, and it still depends on the wall being square to the car.

## Useful Commands

Solve the latest captured run again:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py solve
```

Solve a specific run:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py solve src/perception/scripts/calibration/runs/YYYYMMDD_HHMMSS
```

Refresh validation outputs:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py validate
```

If runtime perception stops using the launch-file `flip=true` behavior, rerun
calibration with:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py wizard --runtime-flip none
```
