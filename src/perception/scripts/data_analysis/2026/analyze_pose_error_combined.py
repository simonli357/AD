#!/usr/bin/env python3
"""
Combined sign/object pose error analysis for long.csv + long2.csv.

The script fits the same first-order correction models used in the controller,
flags gross recording outliers using externally studentized residuals, and writes
a concise report plus plots for the combined data.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


ROOT = Path(__file__).resolve().parent
INPUTS = [ROOT / "long.csv", ROOT / "long2.csv"]
OUT = ROOT / "combined_pose_error_analysis"

OLD = {
    "lon_expl": np.array([-3.519176335, 0.085017553, -0.020975844, 0.024685366]),
    "lat_expl": np.array([-0.095144913, 0.054713559, 0.029940059, 0.016494832]),
    "lon_runtime": np.array([3.375990865, 0.920732200, 0.023003649, -0.026925417]),
    "lat_runtime": np.array([-0.028418154, 0.972237050, -0.048295963, -0.018259780]),
}


def read_rows(paths: list[Path]) -> list[dict[str, float | int | str]]:
    rows = []
    for path in paths:
        with path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            for line, row in enumerate(reader, start=2):
                values = []
                for col in (
                    "estimated_lateral",
                    "estimated_longitudinal",
                    "measured_lateral",
                    "measured_longitudinal",
                ):
                    text = (row.get(col) or "").strip()
                    try:
                        value = float(text) if text else math.nan
                    except ValueError:
                        value = math.nan
                    values.append(value)
                if not all(math.isfinite(value) for value in values):
                    continue
                estimated_lateral, estimated_longitudinal, measured_lateral, measured_longitudinal = values
                rows.append(
                    {
                        "source": path.name,
                        "line": line,
                        "estimated_lateral": estimated_lateral,
                        "estimated_longitudinal": estimated_longitudinal,
                        "measured_lateral": measured_lateral,
                        "measured_longitudinal": measured_longitudinal,
                        "longitudinal_error": estimated_longitudinal - measured_longitudinal,
                        "lateral_error": estimated_lateral - measured_lateral,
                        "abs_measured_lateral": abs(measured_lateral),
                        "abs_estimated_lateral": abs(estimated_lateral),
                    }
                )
    return rows


def matrix(rows: list[dict[str, float | int | str]], cols: list[str]) -> np.ndarray:
    return np.column_stack([np.ones(len(rows))] + [[float(row[col]) for row in rows] for col in cols])


def target(rows: list[dict[str, float | int | str]], col: str) -> np.ndarray:
    return np.array([float(row[col]) for row in rows], dtype=float)


def fit(rows: list[dict[str, float | int | str]], target_col: str, cols: list[str]) -> dict[str, np.ndarray | float]:
    x = matrix(rows, cols)
    y = target(rows, target_col)
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    fitted = x @ beta
    residuals = y - fitted
    n, p = x.shape
    sse = float(residuals @ residuals)
    tss = float((y - y.mean()) @ (y - y.mean()))
    mse = sse / (n - p)
    inv = np.linalg.inv(x.T @ x)
    se = np.sqrt(np.diag(mse * inv))
    p_values = 2.0 * (1.0 - stats.t.cdf(np.abs(beta / se), n - p))
    hat = x @ inv @ x.T
    leverage = np.diag(hat)
    cooks = (residuals**2 / (p * mse)) * (leverage / (1.0 - leverage) ** 2)
    studentized = []
    for index, residual in enumerate(residuals):
        deleted_mse = (sse - (residual**2) / (1.0 - leverage[index])) / max(n - p - 1, 1)
        studentized.append(residual / (math.sqrt(max(deleted_mse, 1e-12)) * math.sqrt(1.0 - leverage[index])))
    r2 = 1.0 - sse / tss
    return {
        "cols": np.array(["offset"] + cols, dtype=object),
        "beta": beta,
        "se": se,
        "p": p_values,
        "fitted": fitted,
        "residuals": residuals,
        "studentized": np.array(studentized),
        "cooks": cooks,
        "adj_r2": 1.0 - (1.0 - r2) * (n - 1.0) / (n - p),
        "rmse": math.sqrt(sse / n),
        "mae": float(np.mean(np.abs(residuals))),
        "median_ae": float(np.median(np.abs(residuals))),
        "max_ae": float(np.max(np.abs(residuals))),
    }


def loocv(rows: list[dict[str, float | int | str]], target_col: str, cols: list[str]) -> tuple[float, float, float, float]:
    errors = []
    for index, row in enumerate(rows):
        train = rows[:index] + rows[index + 1 :]
        beta = fit(train, target_col, cols)["beta"]
        prediction = float((matrix([row], cols) @ beta)[0])
        errors.append(prediction - float(row[target_col]))
    values = np.array(errors, dtype=float)
    abs_values = np.abs(values)
    return (
        float(np.sqrt(np.mean(values**2))),
        float(np.mean(abs_values)),
        float(np.median(abs_values)),
        float(np.max(abs_values)),
    )


def fmt(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


def table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def model_row(name: str, fit_result: dict[str, np.ndarray | float], cv: tuple[float, float, float, float]) -> list[object]:
    beta = fit_result["beta"]
    return [
        name,
        ", ".join(fmt(float(v), 6) for v in beta),
        fmt(float(fit_result["adj_r2"])),
        fmt(float(fit_result["rmse"])),
        fmt(float(fit_result["mae"])),
        fmt(float(fit_result["median_ae"])),
        fmt(cv[0]),
        fmt(cv[1]),
        fmt(cv[3]),
    ]


def evaluate_beta(rows: list[dict[str, float | int | str]], target_col: str, cols: list[str], beta: np.ndarray) -> tuple[float, float, float, float]:
    residuals = target(rows, target_col) - matrix(rows, cols) @ beta
    abs_residuals = np.abs(residuals)
    return (
        float(np.sqrt(np.mean(residuals**2))),
        float(np.mean(abs_residuals)),
        float(np.median(abs_residuals)),
        float(np.max(abs_residuals)),
    )


def plot_error(rows: list[dict[str, float | int | str]], error_col: str, title: str, path: Path) -> None:
    plt.figure(figsize=(8, 5))
    laterals = sorted({float(row["measured_lateral"]) for row in rows})
    for lateral in laterals:
        group = [row for row in rows if float(row["measured_lateral"]) == lateral]
        plt.scatter(
            [float(row["measured_longitudinal"]) for row in group],
            [float(row[error_col]) for row in group],
            s=24,
            label=f"{lateral:g}" if len(laterals) <= 12 else None,
        )
    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("measured longitudinal")
    plt.ylabel(error_col.replace("_", " "))
    plt.title(title)
    if len(laterals) <= 12:
        plt.legend(title="measured lateral", fontsize="small")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def main() -> None:
    rows = read_rows(INPUTS)
    specs = {
        "lon_expl": ("longitudinal_error", ["measured_longitudinal", "measured_lateral", "abs_measured_lateral"]),
        "lat_expl": ("lateral_error", ["measured_longitudinal", "measured_lateral", "abs_measured_lateral"]),
        "lon_runtime": ("measured_longitudinal", ["estimated_longitudinal", "estimated_lateral", "abs_estimated_lateral"]),
        "lat_runtime": ("measured_lateral", ["estimated_lateral", "estimated_longitudinal", "abs_estimated_lateral"]),
    }

    initial_fits = {name: fit(rows, target_col, cols) for name, (target_col, cols) in specs.items()}
    outliers = set()
    for name in ("lon_expl", "lat_expl"):
        fit_result = initial_fits[name]
        for index, studentized in enumerate(fit_result["studentized"]):
            if abs(float(studentized)) > 3.0:
                row = rows[index]
                outliers.add((str(row["source"]), int(row["line"])))

    clean = [row for row in rows if (str(row["source"]), int(row["line"])) not in outliers]
    clean_fits = {name: fit(clean, target_col, cols) for name, (target_col, cols) in specs.items()}
    clean_cv = {name: loocv(clean, target_col, cols) for name, (target_col, cols) in specs.items()}

    OUT.mkdir(exist_ok=True)
    plot_error(clean, "longitudinal_error", "Combined Longitudinal Error", OUT / "combined_longitudinal_error.png")
    plot_error(clean, "lateral_error", "Combined Lateral Error", OUT / "combined_lateral_error.png")

    outlier_rows = []
    for source, line in sorted(outliers):
        row = next(item for item in rows if str(item["source"]) == source and int(item["line"]) == line)
        outlier_rows.append(
            [
                f"{source}:{line}",
                fmt(float(row["measured_lateral"]), 1),
                fmt(float(row["measured_longitudinal"]), 1),
                fmt(float(row["estimated_lateral"]), 1),
                fmt(float(row["estimated_longitudinal"]), 1),
                fmt(float(row["longitudinal_error"]), 2),
                fmt(float(row["lateral_error"]), 2),
            ]
        )

    coef_rows = []
    for name in ("lon_expl", "lat_expl", "lon_runtime", "lat_runtime"):
        old = OLD[name]
        new = clean_fits[name]["beta"]
        for term, old_value, new_value in zip(["offset", "b1", "b2", "b3"], old, new):
            coef_rows.append([name, term, fmt(float(old_value), 6), fmt(float(new_value), 6), fmt(float(new_value - old_value), 6)])

    performance_rows = []
    for name, (target_col, cols) in specs.items():
        old_metrics = evaluate_beta(clean, target_col, cols, OLD[name])
        new_metrics = evaluate_beta(clean, target_col, cols, clean_fits[name]["beta"])
        performance_rows.append(
            [
                name,
                fmt(old_metrics[0]),
                fmt(new_metrics[0]),
                fmt(old_metrics[1]),
                fmt(new_metrics[1]),
                fmt(old_metrics[3]),
                fmt(new_metrics[3]),
            ]
        )

    report = f"""# Combined Pose Error Analysis

Inputs: `{INPUTS[0].name}`, `{INPUTS[1].name}`

Rows:
- `{INPUTS[0].name}`: {sum(1 for row in rows if row["source"] == INPUTS[0].name)}
- `{INPUTS[1].name}`: {sum(1 for row in rows if row["source"] == INPUTS[1].name)}
- combined before outlier removal: {len(rows)}
- combined after outlier removal: {len(clean)}

Outlier rule: remove rows with absolute externally studentized residual greater
than 3 in the explanatory longitudinal or lateral model. This is intended for
obvious recording errors, not ordinary high-leverage endpoint points.

## Removed Outliers

{table(["row", "meas lat", "meas long", "est lat", "est long", "lon err", "lat err"], outlier_rows)}

## Final Coefficients

Coefficient order is `[offset, b1, b2, b3]`.

{table(
        ["model", "coefficients", "adj R2", "RMSE", "MAE", "median AE", "LOOCV RMSE", "LOOCV MAE", "LOOCV max AE"],
        [model_row(name, clean_fits[name], clean_cv[name]) for name in ("lon_expl", "lat_expl", "lon_runtime", "lat_runtime")],
    )}

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

{table(["model", "term", "old", "new", "delta"], coef_rows)}

## Old vs New Performance On Combined Clean Data

{table(["model", "old RMSE", "new RMSE", "old MAE", "new MAE", "old max AE", "new max AE"], performance_rows)}

## Plots

- `combined_longitudinal_error.png`
- `combined_lateral_error.png`
"""
    (OUT / "combined_pose_error_report.md").write_text(report)
    print(f"Wrote {OUT / 'combined_pose_error_report.md'}")
    print(f"Wrote {OUT / 'combined_longitudinal_error.png'}")
    print(f"Wrote {OUT / 'combined_lateral_error.png'}")


if __name__ == "__main__":
    main()
