# Longitudinal Object Pose Error Analysis

Input: `long.csv`

Error convention: `longitudinal_error = estimated_longitudinal - measured_longitudinal`.
Positive error means the object estimate is farther away than the measured ground truth.
All equations use the numeric units already present in the CSV; no unit conversion is applied.

## Data Quality

- Total complete samples used before outlier screening: 27
- Incomplete CSV lines excluded because the estimate is missing: none
- Rows excluded from clean fits: none
- High-influence diagnostic rows retained for review: 18
- Clean-fit sample count: 27

No row exceeded the exclusion threshold of absolute externally studentized residual greater than 3. High-Cook-distance endpoint rows are shown in the influence table for review, but are retained.

## Main Result

The longitudinal error is primarily distance dependent. On the clean data, the
correlation between measured longitudinal distance and longitudinal error is
`0.895`. A constant offset alone is not a good model.

Selected explanatory model:

```text
longitudinal_error = -3.519176 +0.085018*measured_longitudinal -0.020976*measured_lateral +0.024685*abs(measured_lateral)
```

That means the raw estimate behaves approximately like:

```text
estimated_longitudinal = measured_longitudinal + longitudinal_error
```

or:

```text
estimated_longitudinal = -3.519176 +1.085018*measured_longitudinal -0.020976*measured_lateral +0.024685*abs(measured_lateral)
```

Interpretation:

- The distance coefficient is about `+0.0850` error units per measured longitudinal unit, i.e. about a `8.5%` range over-estimate after the offset.
- The fitted offset is `-3.519` units.
- Lateral position matters, but less than distance. The signed lateral coefficient is `-0.0210` error units per lateral unit, and the off-center coefficient is `+0.0247` per `abs(lateral)` unit.

For a runtime correction where measured ground truth is unavailable, use estimated
coordinates as predictors. The best model in this dataset is:

```text
corrected_measured_longitudinal = +3.375991 +0.920732*estimated_longitudinal +0.023004*estimated_lateral -0.026925*abs(estimated_lateral)
```

This model has clean-data LOOCV RMSE `0.835` and
LOOCV MAE `0.721` in CSV units. If you want a
simpler lower-risk correction, use the two-term runtime model from the table
below; it is slightly worse but avoids the absolute-lateral term.

## Error by Lateral Sweep

| measured lateral | n | mean error | std | min | max |
| --- | --- | --- | --- | --- | --- |
| -35 | 9 | 5.7778 | 2.2482 | 3.5000 | 10.0000 |
| 0 | 10 | 4.6000 | 2.1071 | 2.2000 | 8.0000 |
| 35 | 8 | 5.5375 | 3.0047 | 2.3000 | 10.0000 |

## Model Comparison: All Complete Rows

| model | equation | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | max LOOCV AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| offset only | longitudinal_error = +5.27037 | -0.0000 | 2.3647 | 2.0241 | 1.7704 | 2.4556 | 2.1020 | 4.9115 |
| measured longitudinal | longitudinal_error = -2.53505 +0.08075*measured_longitudinal | 0.7931 | 1.0547 | 0.9264 | 0.8544 | 1.1484 | 1.0053 | 2.2569 |
| measured lateral | longitudinal_error = +5.26507 -0.00409*measured_lateral | -0.0376 | 2.3620 | 2.0179 | 1.7651 | 2.5689 | 2.1852 | 5.4228 |
| abs measured lateral | longitudinal_error = +4.60000 +0.03042*abs(measured_lateral) | 0.0092 | 2.3081 | 2.0379 | 2.1000 | 2.4840 | 2.1980 | 4.6063 |
| measured longitudinal + signed lateral | longitudinal_error = -3.04374 +0.08572*measured_longitudinal -0.02166*measured_lateral | 0.8512 | 0.8765 | 0.7665 | 0.6289 | 1.0039 | 0.8692 | 2.1790 |
| measured longitudinal + abs lateral | longitudinal_error = -3.05619 +0.08017*measured_longitudinal +0.02618*abs(measured_lateral) | 0.8224 | 0.9575 | 0.7673 | 0.6591 | 1.0636 | 0.8588 | 1.9950 |
| measured longitudinal + signed lateral + abs lateral | longitudinal_error = -3.51918 +0.08502*measured_longitudinal -0.02098*measured_lateral +0.02469*abs(measured_lateral) | 0.8798 | 0.7712 | 0.6689 | 0.7451 | 0.9149 | 0.7906 | 1.8457 |

No rows were excluded, so the all-row and clean-fit tables should match.

## Model Comparison: Clean Explanatory Fits

| model | equation | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | max LOOCV AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| offset only | longitudinal_error = +5.27037 | -0.0000 | 2.3647 | 2.0241 | 1.7704 | 2.4556 | 2.1020 | 4.9115 |
| measured longitudinal | longitudinal_error = -2.53505 +0.08075*measured_longitudinal | 0.7931 | 1.0547 | 0.9264 | 0.8544 | 1.1484 | 1.0053 | 2.2569 |
| measured lateral | longitudinal_error = +5.26507 -0.00409*measured_lateral | -0.0376 | 2.3620 | 2.0179 | 1.7651 | 2.5689 | 2.1852 | 5.4228 |
| abs measured lateral | longitudinal_error = +4.60000 +0.03042*abs(measured_lateral) | 0.0092 | 2.3081 | 2.0379 | 2.1000 | 2.4840 | 2.1980 | 4.6063 |
| measured longitudinal + signed lateral | longitudinal_error = -3.04374 +0.08572*measured_longitudinal -0.02166*measured_lateral | 0.8512 | 0.8765 | 0.7665 | 0.6289 | 1.0039 | 0.8692 | 2.1790 |
| measured longitudinal + abs lateral | longitudinal_error = -3.05619 +0.08017*measured_longitudinal +0.02618*abs(measured_lateral) | 0.8224 | 0.9575 | 0.7673 | 0.6591 | 1.0636 | 0.8588 | 1.9950 |
| measured longitudinal + signed lateral + abs lateral | longitudinal_error = -3.51918 +0.08502*measured_longitudinal -0.02098*measured_lateral +0.02469*abs(measured_lateral) | 0.8798 | 0.7712 | 0.6689 | 0.7451 | 0.9149 | 0.7906 | 1.8457 |

## Selected Explanatory Coefficients

| term | coef | SE | p | HC3 SE | HC3 p |
| --- | --- | --- | --- | --- | --- |
| offset | -3.519176 | 0.656282 | 0.000019 | 0.784832 | 0.000168 |
| measured_longitudinal | 0.085018 | 0.006290 | 0.000000 | 0.007267 | 0.000000 |
| measured_lateral | -0.020976 | 0.005944 | 0.001796 | 0.007045 | 0.006739 |
| abs(measured_lateral) | 0.024685 | 0.009529 | 0.016354 | 0.009265 | 0.013851 |

## Runtime Correction Fits

| model | equation | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | max LOOCV AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| estimated longitudinal | corrected_measured_longitudinal = +2.47621 +0.92401*estimated_longitudinal | 0.9986 | 0.9752 | 0.8594 | 0.8207 | 1.0624 | 0.9330 | 2.0499 |
| estimated longitudinal + estimated lateral | corrected_measured_longitudinal = +2.88630 +0.91918*estimated_longitudinal +0.01936*estimated_lateral | 0.9990 | 0.8102 | 0.7096 | 0.5812 | 0.9298 | 0.8055 | 2.0028 |
| estimated longitudinal + estimated lateral + abs lateral | corrected_measured_longitudinal = +3.37599 +0.92073*estimated_longitudinal +0.02300*estimated_lateral -0.02693*abs(estimated_lateral) | 0.9992 | 0.7016 | 0.6089 | 0.6411 | 0.8353 | 0.7211 | 1.6508 |

## Selected Runtime Correction Coefficients

| term | coef | SE | p | HC3 SE | HC3 p |
| --- | --- | --- | --- | --- | --- |
| offset | 3.375991 | 0.584545 | 0.000007 | 0.727533 | 0.000114 |
| estimated_longitudinal | 0.920732 | 0.005352 | 0.000000 | 0.006068 | 0.000000 |
| estimated_lateral | 0.023004 | 0.005381 | 0.000284 | 0.006000 | 0.000849 |
| abs(estimated_lateral) | -0.026925 | 0.009726 | 0.010935 | 0.009807 | 0.011524 |

## Top Influence Diagnostics

| line | meas lat | meas long | error | fitted | residual | studentized | Cook D |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 18 | 35.0 | 140.0 | 10.0000 | 8.5131 | 1.4869 | 2.1295 | 0.2372 |
| 17 | 35.0 | 130.0 | 8.9000 | 7.6629 | 1.2371 | 1.6783 | 0.1247 |
| 19 | 0.0 | 55.0 | 2.2000 | 1.1568 | 1.0432 | 1.4201 | 0.1154 |
| 16 | 35.0 | 120.0 | 8.0000 | 6.8128 | 1.1872 | 1.5790 | 0.0935 |
| 10 | -35.0 | 130.0 | 10.0000 | 9.1312 | 0.8688 | 1.1713 | 0.0840 |
| 20 | 0.0 | 60.0 | 2.5000 | 1.5819 | 0.9181 | 1.2198 | 0.0754 |
| 14 | 35.0 | 100.0 | 4.0000 | 5.1124 | -1.1124 | -1.4589 | 0.0734 |
| 3 | -35.0 | 60.0 | 4.0000 | 3.1800 | 0.8200 | 1.0770 | 0.0565 |

## Plots

- `longitudinal_error_analysis/error_vs_measured_longitudinal.png`
- `longitudinal_error_analysis/estimated_vs_measured_longitudinal.png`
- `longitudinal_error_analysis/runtime_correction_residuals.png`
