# Print This Target First

For floor calibration on the car, print this larger target first:

- `charuco_ground_target_a2_large_2x_300dpi.png`

For wall calibration, print this larger target instead:

- `charuco_wall_target_a2_large_2x_300dpi.png`

Print at `Actual size / 100% / no fit to page`. The board should be about
`48 cm x 36 cm`, and the base `100 mm` scale bar should measure about `200 mm`.
Enter the actual measured scale-bar length in the wizard.

The smaller targets are for close-range checks and may be too small on the
floor:

- `charuco_ground_target_letter_300dpi.png`
- `charuco_ground_target_a4_300dpi.png`
- `charuco_wall_target_letter_300dpi.png`
- `charuco_wall_target_a4_300dpi.png`

Use landscape orientation. Do not scale to fit.

After printing:

1. Measure the scale check bar.
2. Tape the paper flat to stiff backing.
3. Place the target flat on the ground in front of the car.
4. Point `+X CAR FORWARD` in the car's forward direction.
5. Point `+Y CAR LEFT` toward the car's left side.
6. Align the target centerline with the car centerline.

Then run:

```bash
python3 ../calibrate_realsense_mount.py wizard
```

For wall calibration:

1. Tape the paper flat to rigid backing on a vertical wall or board.
2. Make the board plane perpendicular to the car centerline.
3. Point `+Z CAR UP` upward.
4. Point `+Y CAR LEFT` toward the car's left side.
5. Align the target vertical centerline with the car centerline.

Then run:

```bash
python3 ../calibrate_realsense_mount.py wizard --target-placement vertical-front
```
