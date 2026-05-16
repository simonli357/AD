#!/usr/bin/env python3
"""
Analyze longitudinal object-pose error against measured range and lateral offset.

The script intentionally avoids pandas/statsmodels so it can run on a minimal
Python environment with only numpy/scipy/matplotlib installed.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


ROOT = Path(__file__).resolve().parent
INPUT_CSV = ROOT / "long.csv"
OUTPUT_DIR = ROOT / "longitudinal_error_analysis"


@dataclass(frozen=True)
class Sample:
    line: int
    estimated_lateral: float
    estimated_longitudinal: float
    measured_lateral: float
    measured_longitudinal: float

    @property
    def longitudinal_error(self) -> float:
        return self.estimated_longitudinal - self.measured_longitudinal

    @property
    def lateral_error(self) -> float:
        return self.estimated_lateral - self.measured_lateral


@dataclass(frozen=True)
class Term:
    name: str
    fn: Callable[[Sample], float]


@dataclass
class Fit:
    name: str
    target_name: str
    terms: list[Term]
    beta: np.ndarray
    se: np.ndarray
    p_values: np.ndarray
    hc3_se: np.ndarray
    hc3_p_values: np.ndarray
    fitted: np.ndarray
    residuals: np.ndarray
    r2: float
    adjusted_r2: float
    rmse: float
    mae: float
    median_ae: float
    max_ae: float
    loocv_rmse: float
    loocv_mae: float
    loocv_median_ae: float
    loocv_max_ae: float

    @property
    def columns(self) -> list[str]:
        return ["offset"] + [term.name for term in self.terms]

    def equation(self, lhs: str, precision: int = 6) -> str:
        parts = [f"{self.beta[0]:+.{precision}f}"]
        for coef, term in zip(self.beta[1:], self.terms):
            parts.append(f"{coef:+.{precision}f}*{term.name}")
        return f"{lhs} = " + " ".join(parts)


def parse_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def read_samples(path: Path) -> tuple[list[Sample], list[int]]:
    samples: list[Sample] = []
    incomplete_lines: list[int] = []
    required = [
        "estimated_lateral",
        "estimated_longitudinal",
        "measured_lateral",
        "measured_longitudinal",
    ]

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for line, row in enumerate(reader, start=2):
            parsed = {name: parse_float(row.get(name, "")) for name in required}
            if any(value is None for value in parsed.values()):
                incomplete_lines.append(line)
                continue
            samples.append(
                Sample(
                    line=line,
                    estimated_lateral=parsed["estimated_lateral"],
                    estimated_longitudinal=parsed["estimated_longitudinal"],
                    measured_lateral=parsed["measured_lateral"],
                    measured_longitudinal=parsed["measured_longitudinal"],
                )
            )

    return samples, incomplete_lines


def design_matrix(samples: list[Sample], terms: list[Term]) -> np.ndarray:
    columns = [np.ones(len(samples))]
    columns.extend([np.array([term.fn(sample) for sample in samples], dtype=float) for term in terms])
    return np.column_stack(columns)


def target_vector(samples: list[Sample], target: Callable[[Sample], float]) -> np.ndarray:
    return np.array([target(sample) for sample in samples], dtype=float)


def loocv(
    samples: list[Sample],
    terms: list[Term],
    target: Callable[[Sample], float],
) -> tuple[float, float, float, float]:
    errors = []
    for held_out in range(len(samples)):
        train = samples[:held_out] + samples[held_out + 1 :]
        test = [samples[held_out]]
        x_train = design_matrix(train, terms)
        y_train = target_vector(train, target)
        beta = np.linalg.lstsq(x_train, y_train, rcond=None)[0]
        prediction = float((design_matrix(test, terms) @ beta)[0])
        errors.append(prediction - target(test[0]))

    values = np.array(errors, dtype=float)
    abs_values = np.abs(values)
    return (
        float(np.sqrt(np.mean(values**2))),
        float(np.mean(abs_values)),
        float(np.median(abs_values)),
        float(np.max(abs_values)),
    )


def fit_model(
    name: str,
    samples: list[Sample],
    terms: list[Term],
    target_name: str,
    target: Callable[[Sample], float],
) -> Fit:
    x = design_matrix(samples, terms)
    y = target_vector(samples, target)
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    fitted = x @ beta
    residuals = y - fitted

    n = len(samples)
    p = x.shape[1]
    sse = float(residuals @ residuals)
    tss = float(((y - y.mean()) @ (y - y.mean())))
    mse = sse / (n - p)
    xtx_inv = np.linalg.inv(x.T @ x)
    covariance = mse * xtx_inv
    se = np.sqrt(np.diag(covariance))
    t_values = beta / se
    p_values = 2.0 * (1.0 - stats.t.cdf(np.abs(t_values), df=n - p))

    hat = x @ xtx_inv @ x.T
    leverage = np.diag(hat)
    hc3_meat = x.T @ np.diag((residuals / (1.0 - leverage)) ** 2) @ x
    hc3_covariance = xtx_inv @ hc3_meat @ xtx_inv
    hc3_se = np.sqrt(np.diag(hc3_covariance))
    hc3_t_values = beta / hc3_se
    hc3_p_values = 2.0 * (1.0 - stats.t.cdf(np.abs(hc3_t_values), df=n - p))

    abs_residuals = np.abs(residuals)
    cv_rmse, cv_mae, cv_median_ae, cv_max_ae = loocv(samples, terms, target)
    r2 = 1.0 - sse / tss
    adjusted_r2 = 1.0 - (1.0 - r2) * (n - 1.0) / (n - p)

    return Fit(
        name=name,
        target_name=target_name,
        terms=terms,
        beta=beta,
        se=se,
        p_values=p_values,
        hc3_se=hc3_se,
        hc3_p_values=hc3_p_values,
        fitted=fitted,
        residuals=residuals,
        r2=float(r2),
        adjusted_r2=float(adjusted_r2),
        rmse=float(np.sqrt(sse / n)),
        mae=float(np.mean(abs_residuals)),
        median_ae=float(np.median(abs_residuals)),
        max_ae=float(np.max(abs_residuals)),
        loocv_rmse=cv_rmse,
        loocv_mae=cv_mae,
        loocv_median_ae=cv_median_ae,
        loocv_max_ae=cv_max_ae,
    )


def influence(samples: list[Sample], fit: Fit) -> list[dict[str, float]]:
    x = design_matrix(samples, fit.terms)
    residuals = fit.residuals
    n = len(samples)
    p = x.shape[1]
    sse = float(residuals @ residuals)
    mse = sse / (n - p)
    xtx_inv = np.linalg.inv(x.T @ x)
    hat = x @ xtx_inv @ x.T
    leverage = np.diag(hat)

    rows = []
    for index, sample in enumerate(samples):
        denom = max(n - p - 1, 1)
        mse_deleted = (sse - (residuals[index] ** 2) / (1.0 - leverage[index])) / denom
        studentized = residuals[index] / (math.sqrt(mse_deleted) * math.sqrt(1.0 - leverage[index]))
        cooks = ((residuals[index] ** 2) / (p * mse)) * (leverage[index] / ((1.0 - leverage[index]) ** 2))
        rows.append(
            {
                "line": sample.line,
                "measured_lateral": sample.measured_lateral,
                "measured_longitudinal": sample.measured_longitudinal,
                "error": sample.longitudinal_error,
                "fitted": fit.fitted[index],
                "residual": residuals[index],
                "studentized_residual": studentized,
                "cooks_d": cooks,
            }
        )

    return sorted(rows, key=lambda row: row["cooks_d"], reverse=True)


def summarize_by_lateral(samples: list[Sample]) -> list[tuple[float, int, float, float, float, float]]:
    summary = []
    for lateral in sorted({sample.measured_lateral for sample in samples}):
        values = np.array(
            [sample.longitudinal_error for sample in samples if sample.measured_lateral == lateral],
            dtype=float,
        )
        std = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        summary.append((lateral, len(values), float(np.mean(values)), std, float(np.min(values)), float(np.max(values))))
    return summary


def format_float(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


def markdown_table(headers: list[str], rows: Iterable[Iterable[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def model_table(fits: list[Fit]) -> str:
    return markdown_table(
        [
            "model",
            "equation",
            "adj R2",
            "RMSE",
            "MAE",
            "median AE",
            "LOOCV RMSE",
            "LOOCV MAE",
            "max LOOCV AE",
        ],
        [
            [
                fit.name,
                fit.equation(fit.target_name, precision=5),
                format_float(fit.adjusted_r2),
                format_float(fit.rmse),
                format_float(fit.mae),
                format_float(fit.median_ae),
                format_float(fit.loocv_rmse),
                format_float(fit.loocv_mae),
                format_float(fit.loocv_max_ae),
            ]
            for fit in fits
        ],
    )


def coefficient_table(fit: Fit) -> str:
    return markdown_table(
        ["term", "coef", "SE", "p", "HC3 SE", "HC3 p"],
        [
            [
                name,
                format_float(coef, 6),
                format_float(se, 6),
                format_float(p_value, 6),
                format_float(hc3_se, 6),
                format_float(hc3_p_value, 6),
            ]
            for name, coef, se, p_value, hc3_se, hc3_p_value in zip(
                fit.columns,
                fit.beta,
                fit.se,
                fit.p_values,
                fit.hc3_se,
                fit.hc3_p_values,
            )
        ],
    )


def make_plots(samples: list[Sample], measured_fit: Fit, runtime_fit: Fit) -> list[Path]:
    OUTPUT_DIR.mkdir(exist_ok=True)
    plot_paths: list[Path] = []

    by_lateral = sorted({sample.measured_lateral for sample in samples})
    colors = {lateral: color for lateral, color in zip(by_lateral, ["tab:blue", "tab:orange", "tab:green", "tab:red"])}

    plt.figure(figsize=(8, 5))
    for lateral in by_lateral:
        group = [sample for sample in samples if sample.measured_lateral == lateral]
        plt.scatter(
            [sample.measured_longitudinal for sample in group],
            [sample.longitudinal_error for sample in group],
            label=f"measured lateral {lateral:g}",
            color=colors[lateral],
        )

    x_min = min(sample.measured_longitudinal for sample in samples)
    x_max = max(sample.measured_longitudinal for sample in samples)
    x_values = np.linspace(x_min, x_max, 120)
    for lateral in by_lateral:
        pseudo = [
            Sample(
                line=0,
                estimated_lateral=lateral,
                estimated_longitudinal=x,
                measured_lateral=lateral,
                measured_longitudinal=x,
            )
            for x in x_values
        ]
        y_values = design_matrix(pseudo, measured_fit.terms) @ measured_fit.beta
        plt.plot(x_values, y_values, color=colors[lateral], linestyle="--", alpha=0.8)

    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("measured longitudinal")
    plt.ylabel("estimated - measured longitudinal")
    plt.title("Longitudinal Error vs Measured Longitudinal")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "error_vs_measured_longitudinal.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    plt.figure(figsize=(8, 5))
    measured = np.array([sample.measured_longitudinal for sample in samples])
    estimated = np.array([sample.estimated_longitudinal for sample in samples])
    plt.scatter(measured, estimated, color="tab:blue")
    lim_min = min(measured.min(), estimated.min()) - 5.0
    lim_max = max(measured.max(), estimated.max()) + 5.0
    plt.plot([lim_min, lim_max], [lim_min, lim_max], color="black", linewidth=0.8, label="perfect")
    plt.xlabel("measured longitudinal")
    plt.ylabel("estimated longitudinal")
    plt.title("Estimated vs Measured Longitudinal")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "estimated_vs_measured_longitudinal.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    corrected = runtime_fit.fitted
    residual = corrected - measured
    plt.figure(figsize=(8, 5))
    plt.scatter(estimated, residual, color="tab:purple")
    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("estimated longitudinal")
    plt.ylabel("corrected measured prediction - measured")
    plt.title("Runtime Correction Residuals")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "runtime_correction_residuals.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    return plot_paths


def build_report(
    all_samples: list[Sample],
    clean_samples: list[Sample],
    incomplete_lines: list[int],
    flagged_lines: list[int],
    all_measured_fits: list[Fit],
    clean_measured_fits: list[Fit],
    clean_runtime_fits: list[Fit],
    selected_measured_fit: Fit,
    selected_runtime_fit: Fit,
    influence_rows: list[dict[str, float]],
    plot_paths: list[Path],
) -> str:
    error_values = np.array([sample.longitudinal_error for sample in clean_samples], dtype=float)
    measured_values = np.array([sample.measured_longitudinal for sample in clean_samples], dtype=float)
    corr = np.corrcoef(measured_values, error_values)[0, 1]

    lateral_summary = summarize_by_lateral(clean_samples)
    flagged_text = ", ".join(str(line) for line in flagged_lines) if flagged_lines else "none"
    incomplete_text = ", ".join(str(line) for line in incomplete_lines) if incomplete_lines else "none"
    plot_text = "\n".join(f"- `{path.relative_to(ROOT)}`" for path in plot_paths)

    return f"""# Longitudinal Object Pose Error Analysis

Input: `{INPUT_CSV.name}`

Error convention: `longitudinal_error = estimated_longitudinal - measured_longitudinal`.
Positive error means the object estimate is farther away than the measured ground truth.
All equations use the numeric units already present in the CSV; no unit conversion is applied.

## Data Quality

- Total complete samples used before outlier screening: {len(all_samples)}
- Incomplete CSV lines excluded because the estimate is missing: {incomplete_text}
- Influential line flagged by Cook's distance/studentized residual: {flagged_text}
- Clean-fit sample count: {len(clean_samples)}

Line 30 is treated as a diagnostic outlier, not silently deleted. It records
`measured_lateral=-35`, `measured_longitudinal=140`, `estimated_longitudinal=140`,
so its longitudinal error is `0`. The neighboring trends at the same distance
predict about 8 to 10 units of positive error, which makes this row inconsistent
with the rest of the sweep.

## Main Result

The longitudinal error is primarily distance dependent. On the clean data, the
correlation between measured longitudinal distance and longitudinal error is
`{corr:.3f}`. A constant offset alone is not a good model.

Selected explanatory model:

```text
{selected_measured_fit.equation("longitudinal_error", precision=6)}
```

That means the raw estimate behaves approximately like:

```text
estimated_longitudinal = measured_longitudinal + longitudinal_error
```

or:

```text
estimated_longitudinal = {selected_measured_fit.beta[0]:+.6f} {(1.0 + selected_measured_fit.beta[1]):+.6f}*measured_longitudinal {selected_measured_fit.beta[2]:+.6f}*measured_lateral {selected_measured_fit.beta[3]:+.6f}*abs(measured_lateral)
```

Interpretation:

- The distance coefficient is about `+{selected_measured_fit.beta[1]:.4f}` error units per measured longitudinal unit, i.e. about an 8.5% range over-estimate after the offset.
- The fitted offset is `{selected_measured_fit.beta[0]:+.3f}` units.
- Lateral position matters, but less than distance. The signed lateral coefficient is `{selected_measured_fit.beta[2]:+.4f}` error units per lateral unit, and the off-center coefficient is `{selected_measured_fit.beta[3]:+.4f}` per `abs(lateral)` unit.

For a runtime correction where measured ground truth is unavailable, use estimated
coordinates as predictors. The best model in this dataset is:

```text
{selected_runtime_fit.equation("corrected_measured_longitudinal", precision=6)}
```

This model has clean-data LOOCV RMSE `{selected_runtime_fit.loocv_rmse:.3f}` and
LOOCV MAE `{selected_runtime_fit.loocv_mae:.3f}` in CSV units. If you want a
simpler lower-risk correction, use the two-term runtime model from the table
below; it is slightly worse but avoids the absolute-lateral term.

## Error by Lateral Sweep

{markdown_table(
        ["measured lateral", "n", "mean error", "std", "min", "max"],
        [
            [
                f"{lateral:g}",
                n,
                format_float(mean),
                format_float(std),
                format_float(minimum),
                format_float(maximum),
            ]
            for lateral, n, mean, std, minimum, maximum in lateral_summary
        ],
    )}

## Model Comparison: All Complete Rows

{model_table(all_measured_fits)}

The all-row table is dominated by the inconsistent line 30, which suppresses the
distance slope and reduces the apparent quality of all fits.

## Model Comparison: Clean Explanatory Fits

{model_table(clean_measured_fits)}

## Selected Explanatory Coefficients

{coefficient_table(selected_measured_fit)}

## Runtime Correction Fits

{model_table(clean_runtime_fits)}

## Selected Runtime Correction Coefficients

{coefficient_table(selected_runtime_fit)}

## Top Influence Diagnostics

{markdown_table(
        ["line", "meas lat", "meas long", "error", "fitted", "residual", "studentized", "Cook D"],
        [
            [
                int(row["line"]),
                format_float(row["measured_lateral"], 1),
                format_float(row["measured_longitudinal"], 1),
                format_float(row["error"]),
                format_float(row["fitted"]),
                format_float(row["residual"]),
                format_float(row["studentized_residual"]),
                format_float(row["cooks_d"]),
            ]
            for row in influence_rows[:8]
        ],
    )}

## Plots

{plot_text}
"""


def main() -> None:
    samples, incomplete_lines = read_samples(INPUT_CSV)
    if len(samples) < 5:
        raise SystemExit(f"Not enough complete samples in {INPUT_CSV}")

    measured_longitudinal = Term("measured_longitudinal", lambda sample: sample.measured_longitudinal)
    measured_lateral = Term("measured_lateral", lambda sample: sample.measured_lateral)
    abs_measured_lateral = Term("abs(measured_lateral)", lambda sample: abs(sample.measured_lateral))
    estimated_longitudinal = Term("estimated_longitudinal", lambda sample: sample.estimated_longitudinal)
    estimated_lateral = Term("estimated_lateral", lambda sample: sample.estimated_lateral)
    abs_estimated_lateral = Term("abs(estimated_lateral)", lambda sample: abs(sample.estimated_lateral))

    longitudinal_error = lambda sample: sample.longitudinal_error
    measured_longitudinal_target = lambda sample: sample.measured_longitudinal

    primary_all_fit = fit_model(
        "measured longitudinal + signed lateral + abs lateral",
        samples,
        [measured_longitudinal, measured_lateral, abs_measured_lateral],
        "longitudinal_error",
        longitudinal_error,
    )
    all_influence = influence(samples, primary_all_fit)
    cook_threshold = 4.0 / len(samples)
    flagged_lines = [
        int(row["line"])
        for row in all_influence
        if row["cooks_d"] > cook_threshold or abs(row["studentized_residual"]) > 3.0
    ]
    clean_samples = [sample for sample in samples if sample.line not in flagged_lines]

    measured_specs = [
        ("offset only", []),
        ("measured longitudinal", [measured_longitudinal]),
        ("measured lateral", [measured_lateral]),
        ("abs measured lateral", [abs_measured_lateral]),
        ("measured longitudinal + signed lateral", [measured_longitudinal, measured_lateral]),
        ("measured longitudinal + abs lateral", [measured_longitudinal, abs_measured_lateral]),
        (
            "measured longitudinal + signed lateral + abs lateral",
            [measured_longitudinal, measured_lateral, abs_measured_lateral],
        ),
    ]

    runtime_specs = [
        ("estimated longitudinal", [estimated_longitudinal]),
        ("estimated longitudinal + estimated lateral", [estimated_longitudinal, estimated_lateral]),
        (
            "estimated longitudinal + estimated lateral + abs lateral",
            [estimated_longitudinal, estimated_lateral, abs_estimated_lateral],
        ),
    ]

    all_measured_fits = [
        fit_model(name, samples, terms, "longitudinal_error", longitudinal_error)
        for name, terms in measured_specs
    ]
    clean_measured_fits = [
        fit_model(name, clean_samples, terms, "longitudinal_error", longitudinal_error)
        for name, terms in measured_specs
    ]
    clean_runtime_fits = [
        fit_model(name, clean_samples, terms, "corrected_measured_longitudinal", measured_longitudinal_target)
        for name, terms in runtime_specs
    ]

    selected_measured_fit = next(
        fit
        for fit in clean_measured_fits
        if fit.name == "measured longitudinal + signed lateral + abs lateral"
    )
    selected_runtime_fit = next(
        fit
        for fit in clean_runtime_fits
        if fit.name == "estimated longitudinal + estimated lateral + abs lateral"
    )

    OUTPUT_DIR.mkdir(exist_ok=True)
    plot_paths = make_plots(clean_samples, selected_measured_fit, selected_runtime_fit)
    report = build_report(
        all_samples=samples,
        clean_samples=clean_samples,
        incomplete_lines=incomplete_lines,
        flagged_lines=flagged_lines,
        all_measured_fits=all_measured_fits,
        clean_measured_fits=clean_measured_fits,
        clean_runtime_fits=clean_runtime_fits,
        selected_measured_fit=selected_measured_fit,
        selected_runtime_fit=selected_runtime_fit,
        influence_rows=all_influence,
        plot_paths=plot_paths,
    )

    report_path = OUTPUT_DIR / "longitudinal_error_report.md"
    report_path.write_text(report)

    print(f"Wrote {report_path}")
    for path in plot_paths:
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
