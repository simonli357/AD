#!/usr/bin/env python3
"""
Analyze lateral object-pose error against measured distance and lateral offset.

The script mirrors analyze_longitudinal_error.py, but targets lateral error:

    lateral_error = estimated_lateral - measured_lateral

It uses only numpy/scipy/matplotlib so the report can be regenerated on a
minimal environment.
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
OUTPUT_DIR = ROOT / "lateral_error_analysis"


@dataclass(frozen=True)
class Sample:
    line: int
    estimated_lateral: float
    estimated_longitudinal: float
    measured_lateral: float
    measured_longitudinal: float

    @property
    def lateral_error(self) -> float:
        return self.estimated_lateral - self.measured_lateral

    @property
    def longitudinal_error(self) -> float:
        return self.estimated_longitudinal - self.measured_longitudinal


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
    return parsed if math.isfinite(parsed) else None


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
        beta = np.linalg.lstsq(design_matrix(train, terms), target_vector(train, target), rcond=None)[0]
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
    p_values = 2.0 * (1.0 - stats.t.cdf(np.abs(beta / se), df=n - p))

    hat = x @ xtx_inv @ x.T
    leverage = np.diag(hat)
    hc3_meat = x.T @ np.diag((residuals / (1.0 - leverage)) ** 2) @ x
    hc3_covariance = xtx_inv @ hc3_meat @ xtx_inv
    hc3_se = np.sqrt(np.diag(hc3_covariance))
    hc3_p_values = 2.0 * (1.0 - stats.t.cdf(np.abs(beta / hc3_se), df=n - p))

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
                "error": sample.lateral_error,
                "fitted": fit.fitted[index],
                "residual": residuals[index],
                "studentized_residual": studentized,
                "cooks_d": cooks,
            }
        )
    return sorted(rows, key=lambda row: row["cooks_d"], reverse=True)


def markdown_table(headers: list[str], rows: Iterable[Iterable[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def format_float(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


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


def summarize_by_lateral(samples: list[Sample]) -> list[tuple[float, int, float, float, float, float]]:
    summary = []
    for lateral in sorted({sample.measured_lateral for sample in samples}):
        values = np.array([sample.lateral_error for sample in samples if sample.measured_lateral == lateral])
        std = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        summary.append((lateral, len(values), float(np.mean(values)), std, float(np.min(values)), float(np.max(values))))
    return summary


def make_plots(samples: list[Sample], measured_fit: Fit, runtime_fit: Fit) -> list[Path]:
    OUTPUT_DIR.mkdir(exist_ok=True)
    plot_paths: list[Path] = []

    by_lateral = sorted({sample.measured_lateral for sample in samples})
    colors = {lateral: color for lateral, color in zip(by_lateral, ["tab:blue", "tab:orange", "tab:green"])}

    plt.figure(figsize=(8, 5))
    for lateral in by_lateral:
        group = [sample for sample in samples if sample.measured_lateral == lateral]
        plt.scatter(
            [sample.measured_longitudinal for sample in group],
            [sample.lateral_error for sample in group],
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
    plt.ylabel("estimated - measured lateral")
    plt.title("Lateral Error vs Measured Longitudinal")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "lateral_error_vs_measured_longitudinal.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    plt.figure(figsize=(8, 5))
    for longitudinal in sorted({sample.measured_longitudinal for sample in samples}):
        group = [sample for sample in samples if sample.measured_longitudinal == longitudinal]
        if len(group) < 2:
            continue
        plt.plot(
            [sample.measured_lateral for sample in group],
            [sample.lateral_error for sample in group],
            marker="o",
            linewidth=0.8,
            alpha=0.45,
        )
    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("measured lateral")
    plt.ylabel("estimated - measured lateral")
    plt.title("Lateral Error vs Measured Lateral")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "lateral_error_vs_measured_lateral.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    measured = np.array([sample.measured_lateral for sample in samples])
    estimated = np.array([sample.estimated_lateral for sample in samples])
    corrected = runtime_fit.fitted
    residual = corrected - measured
    plt.figure(figsize=(8, 5))
    plt.scatter(estimated, residual, color="tab:purple")
    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("estimated lateral")
    plt.ylabel("corrected measured prediction - measured")
    plt.title("Runtime Lateral Correction Residuals")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = OUTPUT_DIR / "runtime_lateral_correction_residuals.png"
    plt.savefig(path, dpi=180)
    plt.close()
    plot_paths.append(path)

    return plot_paths


def build_report(
    samples: list[Sample],
    incomplete_lines: list[int],
    excluded_lines: list[int],
    influential_lines: list[int],
    explanatory_fits: list[Fit],
    runtime_fits: list[Fit],
    selected_explanatory_fit: Fit,
    selected_runtime_fit: Fit,
    influence_rows: list[dict[str, float]],
    plot_paths: list[Path],
) -> str:
    lateral_errors = np.array([sample.lateral_error for sample in samples])
    measured_longitudinal = np.array([sample.measured_longitudinal for sample in samples])
    measured_lateral = np.array([sample.measured_lateral for sample in samples])
    corr_long = np.corrcoef(measured_longitudinal, lateral_errors)[0, 1]
    corr_lat = np.corrcoef(measured_lateral, lateral_errors)[0, 1]
    incomplete_text = ", ".join(str(line) for line in incomplete_lines) if incomplete_lines else "none"
    excluded_text = ", ".join(str(line) for line in excluded_lines) if excluded_lines else "none"
    influential_text = ", ".join(str(line) for line in influential_lines) if influential_lines else "none"
    plot_text = "\n".join(f"- `{path.relative_to(ROOT)}`" for path in plot_paths)

    return f"""# Lateral Object Pose Error Analysis

Input: `{INPUT_CSV.name}`

Error convention: `lateral_error = estimated_lateral - measured_lateral`.
Positive error means the estimated lateral coordinate is numerically larger than
ground truth in the CSV coordinate system.

## Data Sufficiency

This dataset is enough for a first-order lateral-error analysis. It has
`{len(samples)}` complete samples across three lateral positions: `-35`, `0`,
and `+35`, with longitudinal coverage from `{min(sample.measured_longitudinal for sample in samples):g}`
to `{max(sample.measured_longitudinal for sample in samples):g}`.

It is not enough to prove a high-confidence general lateral model. The lateral
axis only has three ground-truth levels, so nonlinear lateral effects, behavior
beyond `abs(lateral)=35`, and object-class-specific effects are not identifiable
from this CSV alone.

## Data Quality

- Incomplete CSV lines excluded because the estimate is missing: {incomplete_text}
- Rows excluded from fits: {excluded_text}
- High-influence diagnostic rows retained for review: {influential_text}

## Main Result

Lateral error depends on both longitudinal distance and signed lateral position.
The correlation with measured longitudinal distance is `{corr_long:.3f}` and the
correlation with measured lateral position is `{corr_lat:.3f}`.

Selected explanatory model:

```text
{selected_explanatory_fit.equation("lateral_error", precision=6)}
```

Interpretation:

- The lateral estimate has a distance-dependent drift of about `{selected_explanatory_fit.beta[1]:.4f}` lateral-error units per longitudinal unit.
- The signed-lateral term is about `{selected_explanatory_fit.beta[2]:.4f}` error units per lateral unit.
- The off-center term is about `{selected_explanatory_fit.beta[3]:.4f}` per `abs(lateral)` unit and is borderline significant, so treat it as useful but less certain.

For a runtime correction where measured ground truth is unavailable:

```text
{selected_runtime_fit.equation("corrected_measured_lateral", precision=6)}
```

This runtime model has LOOCV RMSE `{selected_runtime_fit.loocv_rmse:.3f}` and
LOOCV MAE `{selected_runtime_fit.loocv_mae:.3f}` in CSV units.

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
            for lateral, n, mean, std, minimum, maximum in summarize_by_lateral(samples)
        ],
    )}

## Explanatory Model Comparison

{model_table(explanatory_fits)}

## Selected Explanatory Coefficients

{coefficient_table(selected_explanatory_fit)}

## Runtime Correction Fits

{model_table(runtime_fits)}

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

    lateral_error = lambda sample: sample.lateral_error
    measured_lateral_target = lambda sample: sample.measured_lateral

    explanatory_specs = [
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
        ("estimated lateral", [estimated_lateral]),
        ("estimated lateral + estimated longitudinal", [estimated_lateral, estimated_longitudinal]),
        (
            "estimated lateral + estimated longitudinal + abs lateral",
            [estimated_lateral, estimated_longitudinal, abs_estimated_lateral],
        ),
    ]

    explanatory_fits = [
        fit_model(name, samples, terms, "lateral_error", lateral_error)
        for name, terms in explanatory_specs
    ]
    selected_explanatory_fit = next(
        fit
        for fit in explanatory_fits
        if fit.name == "measured longitudinal + signed lateral + abs lateral"
    )
    influence_rows = influence(samples, selected_explanatory_fit)
    cook_threshold = 4.0 / len(samples)
    excluded_lines: list[int] = []
    influential_lines = [
        int(row["line"])
        for row in influence_rows
        if row["cooks_d"] > cook_threshold or abs(row["studentized_residual"]) > 3.0
    ]

    fit_samples = [sample for sample in samples if sample.line not in excluded_lines]
    if len(fit_samples) != len(samples):
        explanatory_fits = [
            fit_model(name, fit_samples, terms, "lateral_error", lateral_error)
            for name, terms in explanatory_specs
        ]
        selected_explanatory_fit = next(
            fit
            for fit in explanatory_fits
            if fit.name == "measured longitudinal + signed lateral + abs lateral"
        )

    runtime_fits = [
        fit_model(name, fit_samples, terms, "corrected_measured_lateral", measured_lateral_target)
        for name, terms in runtime_specs
    ]
    selected_runtime_fit = next(
        fit
        for fit in runtime_fits
        if fit.name == "estimated lateral + estimated longitudinal + abs lateral"
    )

    OUTPUT_DIR.mkdir(exist_ok=True)
    plot_paths = make_plots(fit_samples, selected_explanatory_fit, selected_runtime_fit)
    report = build_report(
        samples=fit_samples,
        incomplete_lines=incomplete_lines,
        excluded_lines=excluded_lines,
        influential_lines=influential_lines,
        explanatory_fits=explanatory_fits,
        runtime_fits=runtime_fits,
        selected_explanatory_fit=selected_explanatory_fit,
        selected_runtime_fit=selected_runtime_fit,
        influence_rows=influence_rows,
        plot_paths=plot_paths,
    )
    report_path = OUTPUT_DIR / "lateral_error_report.md"
    report_path.write_text(report)
    print(f"Wrote {report_path}")
    for path in plot_paths:
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
