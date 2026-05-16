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

It is an A2 landscape target. Print it at `Actual size / 100% / no fit to
page`. Its board is about `48 cm x 36 cm`; the base `100 mm` scale bar should
measure about `200 mm`. Enter the actual measured scale-bar length when the
wizard asks.

The smaller files are kept for close-range checks, but they are usually too
small for floor calibration with the car camera:

- `assets/charuco_ground_target_letter_300dpi.png`
- `assets/charuco_ground_target_a4_300dpi.png`

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

Place the target flat on the ground in front of the car:

- `+X CAR FORWARD` points in the car's forward direction.
- `+Y CAR LEFT` points toward the car's left side.
- The target centerline is aligned with the car centerline.
- The car and target do not move during capture.

Yaw accuracy depends mostly on this physical alignment.

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

If the checker sees too few corners at the default runtime resolution, capture
at a higher RealSense color resolution:

```bash
python3 src/perception/scripts/calibration/calibrate_realsense_mount.py capture --auto --no-window --color-width 1920 --color-height 1080
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
