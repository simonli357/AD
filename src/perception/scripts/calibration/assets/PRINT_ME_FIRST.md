# Print This Target First

Print one of these files at `Actual size / 100% / no fit to page`:

- `charuco_ground_target_letter_300dpi.png`
- `charuco_ground_target_a4_300dpi.png`

Use landscape orientation. Do not scale to fit.

After printing:

1. Measure the `100 mm SCALE CHECK` bar.
2. Tape the paper flat to stiff backing.
3. Place the target flat on the ground in front of the car.
4. Point `+X CAR FORWARD` in the car's forward direction.
5. Point `+Y CAR LEFT` toward the car's left side.
6. Align the target centerline with the car centerline.

Then run:

```bash
python3 ../calibrate_realsense_mount.py wizard
```
