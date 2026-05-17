# Combined Pose Error Analysis

Inputs: `long.csv`, `long2.csv`

Rows:
- `long.csv`: 27
- `long2.csv`: 41
- combined before outlier removal: 68
- combined after outlier removal: 67

Outlier rule: remove rows with absolute externally studentized residual greater
than 3 in the explanatory longitudinal or lateral model. This is intended for
obvious recording errors, not ordinary high-leverage endpoint points.

## Removed Outliers

| row | meas lat | meas long | est lat | est long | lon err | lat err |
| --- | --- | --- | --- | --- | --- | --- |
| long2.csv:12 | -15.0 | 150.0 | -7.0 | 147.2 | -2.80 | 8.00 |

## Final Coefficients

Coefficient order is `[offset, b1, b2, b3]`.

| model | coefficients | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | LOOCV max AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| lon_expl | -4.127679, 0.090712, -0.019943, 0.024751 | 0.9220 | 0.7882 | 0.6405 | 0.5282 | 0.8485 | 0.6859 | 2.4631 |
| lat_expl | 1.577041, 0.042796, 0.049274, -0.013697 | 0.8284 | 0.7772 | 0.6350 | 0.5137 | 0.8313 | 0.6777 | 2.1865 |
| lon_runtime | 3.834467, 0.915505, 0.020577, -0.021821 | 0.9993 | 0.7155 | 0.5714 | 0.5455 | 0.7715 | 0.6124 | 2.2281 |
| lat_runtime | -1.700339, 0.950035, -0.037676, 0.016156 | 0.9993 | 0.7256 | 0.5899 | 0.4876 | 0.7755 | 0.6296 | 2.0893 |

Model definitions:

```text
lon_expl:
  longitudinal_error = offset
    + b1*measured_longitudinal
    + b2*measured_lateral
    + b3*abs(measured_lateral)

lat_expl:
  lateral_error = offset
    + b1*measured_longitudinal
    + b2*measured_lateral
    + b3*abs(measured_lateral)

lon_runtime:
  corrected_measured_longitudinal = offset
    + b1*estimated_longitudinal
    + b2*estimated_lateral
    + b3*abs(estimated_lateral)

lat_runtime:
  corrected_measured_lateral = offset
    + b1*estimated_lateral
    + b2*estimated_longitudinal
    + b3*abs(estimated_lateral)
```

## Old vs New Coefficients

| model | term | old | new | delta |
| --- | --- | --- | --- | --- |
| lon_expl | offset | -3.519176 | -4.127679 | -0.608503 |
| lon_expl | b1 | 0.085018 | 0.090712 | 0.005694 |
| lon_expl | b2 | -0.020976 | -0.019943 | 0.001033 |
| lon_expl | b3 | 0.024685 | 0.024751 | 0.000066 |
| lat_expl | offset | -0.095145 | 1.577041 | 1.672186 |
| lat_expl | b1 | 0.054714 | 0.042796 | -0.011917 |
| lat_expl | b2 | 0.029940 | 0.049274 | 0.019334 |
| lat_expl | b3 | 0.016495 | -0.013697 | -0.030192 |
| lon_runtime | offset | 3.375991 | 3.834467 | 0.458477 |
| lon_runtime | b1 | 0.920732 | 0.915505 | -0.005228 |
| lon_runtime | b2 | 0.023004 | 0.020577 | -0.002427 |
| lon_runtime | b3 | -0.026925 | -0.021821 | 0.005105 |
| lat_runtime | offset | -0.028418 | -1.700339 | -1.671921 |
| lat_runtime | b1 | 0.972237 | 0.950035 | -0.022202 |
| lat_runtime | b2 | -0.048296 | -0.037676 | 0.010620 |
| lat_runtime | b3 | -0.018260 | 0.016156 | 0.034416 |

## Old vs New Performance On Combined Clean Data

| model | old RMSE | new RMSE | old MAE | new MAE | old max AE | new max AE |
| --- | --- | --- | --- | --- | --- | --- |
| lon_expl | 0.8037 | 0.7882 | 0.6519 | 0.6405 | 2.4835 | 2.2862 |
| lat_expl | 1.5079 | 0.7772 | 1.0856 | 0.6350 | 4.1293 | 2.1033 |
| lon_runtime | 0.7380 | 0.7155 | 0.5917 | 0.5714 | 2.1193 | 2.0609 |
| lat_runtime | 1.5017 | 0.7256 | 1.0795 | 0.5899 | 4.2550 | 2.0138 |

## Plots

- `combined_longitudinal_error.png`
- `combined_lateral_error.png`
