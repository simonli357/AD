# Lateral Object Pose Error Analysis

Input: `long.csv`

Error convention: `lateral_error = estimated_lateral - measured_lateral`.
Positive error means the estimated lateral coordinate is numerically larger than
ground truth in the CSV coordinate system.

## Data Sufficiency

This dataset is enough for a first-order lateral-error analysis. It has
`27` complete samples across three lateral positions: `-35`, `0`,
and `+35`, with longitudinal coverage from `55`
to `140`.

It is not enough to prove a high-confidence general lateral model. The lateral
axis only has three ground-truth levels, so nonlinear lateral effects, behavior
beyond `abs(lateral)=35`, and object-class-specific effects are not identifiable
from this CSV alone.

## Data Quality

- Incomplete CSV lines excluded because the estimate is missing: none
- Rows excluded from fits: none
- High-influence diagnostic rows retained for review: 10, 11

## Main Result

Lateral error depends on both longitudinal distance and signed lateral position.
The correlation with measured longitudinal distance is `0.836` and the
correlation with measured lateral position is `0.583`.

Selected explanatory model:

```text
lateral_error = -0.095145 +0.054714*measured_longitudinal +0.029940*measured_lateral +0.016495*abs(measured_lateral)
```

Interpretation:

- The lateral estimate has a distance-dependent drift of about `0.0547` lateral-error units per longitudinal unit.
- The signed-lateral term is about `0.0299` error units per lateral unit.
- The off-center term is about `0.0165` per `abs(lateral)` unit and is borderline significant, so treat it as useful but less certain.

For a runtime correction where measured ground truth is unavailable:

```text
corrected_measured_lateral = -0.028418 +0.972237*estimated_lateral -0.048296*estimated_longitudinal -0.018260*abs(estimated_lateral)
```

This runtime model has LOOCV RMSE `0.754` and
LOOCV MAE `0.472` in CSV units.

## Error by Lateral Sweep

| measured lateral | n | mean error | std | min | max |
| --- | --- | --- | --- | --- | --- |
| -35 | 9 | 4.3889 | 0.9955 | 3.0000 | 5.9000 |
| 0 | 10 | 5.1300 | 1.7480 | 2.9000 | 7.8000 |
| 35 | 8 | 7.2750 | 2.0247 | 4.3000 | 10.1000 |

## Explanatory Model Comparison

| model | equation | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | max LOOCV AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| offset only | lateral_error = +5.51852 | -0.0000 | 1.9418 | 1.5997 | 1.3185 | 2.0164 | 1.6613 | 4.7577 |
| measured longitudinal | lateral_error = -0.46995 +0.06195*measured_longitudinal | 0.6873 | 1.0648 | 0.7703 | 0.4140 | 1.1642 | 0.8332 | 3.7467 |
| measured lateral | lateral_error = +5.57140 +0.04079*measured_lateral | 0.3132 | 1.5779 | 1.3301 | 1.1437 | 1.7023 | 1.4348 | 3.4471 |
| abs measured lateral | lateral_error = +5.13000 +0.01763*abs(measured_lateral) | -0.0155 | 1.9188 | 1.6163 | 1.4700 | 2.0649 | 1.7438 | 4.6250 |
| measured longitudinal + signed lateral | lateral_error = +0.22254 +0.05518*measured_longitudinal +0.02948*measured_lateral | 0.8574 | 0.7044 | 0.5590 | 0.4792 | 0.8260 | 0.6406 | 2.6499 |
| measured longitudinal + abs lateral | lateral_error = -0.75599 +0.06163*measured_longitudinal +0.01437*abs(measured_lateral) | 0.6912 | 1.0367 | 0.6697 | 0.2662 | 1.1596 | 0.7437 | 4.0278 |
| measured longitudinal + signed lateral + abs lateral | lateral_error = -0.09514 +0.05471*measured_longitudinal +0.02994*measured_lateral +0.01649*abs(measured_lateral) | 0.8745 | 0.6470 | 0.4234 | 0.1877 | 0.7899 | 0.5077 | 2.9312 |

## Selected Explanatory Coefficients

| term | coef | SE | p | HC3 SE | HC3 p |
| --- | --- | --- | --- | --- | --- |
| offset | -0.095145 | 0.550625 | 0.864325 | 0.757273 | 0.901107 |
| measured_longitudinal | 0.054714 | 0.005278 | 0.000000 | 0.007931 | 0.000000 |
| measured_lateral | 0.029940 | 0.004987 | 0.000004 | 0.007740 | 0.000780 |
| abs(measured_lateral) | 0.016495 | 0.007995 | 0.050567 | 0.006708 | 0.021874 |

## Runtime Correction Fits

| model | equation | adj R2 | RMSE | MAE | median AE | LOOCV RMSE | LOOCV MAE | max LOOCV AE |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| estimated lateral | corrected_measured_lateral = -5.34096 +0.95795*estimated_lateral | 0.9969 | 1.5138 | 1.2856 | 0.9952 | 1.6340 | 1.3873 | 3.2200 |
| estimated lateral + estimated longitudinal | corrected_measured_lateral = -0.36051 +0.96977*estimated_lateral -0.04935*estimated_longitudinal | 0.9994 | 0.6719 | 0.5260 | 0.4254 | 0.7945 | 0.6048 | 2.6629 |
| estimated lateral + estimated longitudinal + abs lateral | corrected_measured_lateral = -0.02842 +0.97224*estimated_lateral -0.04830*estimated_longitudinal -0.01826*abs(estimated_lateral) | 0.9994 | 0.6132 | 0.3927 | 0.1850 | 0.7539 | 0.4721 | 2.9062 |

## Selected Runtime Correction Coefficients

| term | coef | SE | p | HC3 SE | HC3 p |
| --- | --- | --- | --- | --- | --- |
| offset | -0.028418 | 0.510902 | 0.956122 | 0.721835 | 0.968936 |
| estimated_lateral | 0.972237 | 0.004703 | 0.000000 | 0.007765 | 0.000000 |
| estimated_longitudinal | -0.048296 | 0.004678 | 0.000000 | 0.007456 | 0.000001 |
| abs(estimated_lateral) | -0.018260 | 0.008501 | 0.042478 | 0.006752 | 0.012653 |

## Top Influence Diagnostics

| line | meas lat | meas long | error | fitted | residual | studentized | Cook D |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | -35.0 | 130.0 | 4.2000 | 6.5470 | -2.3470 | -5.8487 | 0.8710 |
| 11 | 35.0 | 70.0 | 4.3000 | 5.3600 | -1.0600 | -1.7598 | 0.1712 |
| 18 | 35.0 | 140.0 | 10.1000 | 9.1900 | 0.9100 | 1.4835 | 0.1262 |
| 5 | -35.0 | 80.0 | 4.9000 | 3.8114 | 1.0886 | 1.7221 | 0.0909 |
| 12 | 35.0 | 80.0 | 5.1000 | 5.9072 | -0.8072 | -1.2734 | 0.0754 |
| 17 | 35.0 | 130.0 | 9.3000 | 8.6428 | 0.6572 | 1.0241 | 0.0500 |
| 2 | -35.0 | 55.0 | 3.0000 | 2.4435 | 0.5565 | 0.8735 | 0.0431 |
| 14 | 35.0 | 100.0 | 6.4000 | 7.0014 | -0.6014 | -0.9146 | 0.0305 |

## Plots

- `lateral_error_analysis/lateral_error_vs_measured_longitudinal.png`
- `lateral_error_analysis/lateral_error_vs_measured_lateral.png`
- `lateral_error_analysis/runtime_lateral_correction_residuals.png`
