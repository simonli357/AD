#!/usr/bin/env python3
from pathlib import Path
import argparse
import math
import re
import sys


# Edit these after steering calibration. Values are signed controller/acados degrees:
# positive turns left, negative turns right.
STEERING_MAX_RIGHT_DEG = -24.14
STEERING_MAX_LEFT_DEG = 25.3


ASSIGNMENT_TEMPLATE = r"(^\s*{name}\[1\]\s*=\s*)[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?(\s*;)"


def format_rad(value):
    return f"{value:.10f}"


def replace_bound(text, name, value):
    pattern = re.compile(ASSIGNMENT_TEMPLATE.format(name=name), re.MULTILINE)
    updated, count = pattern.subn(rf"\g<1>{format_rad(value)}\g<2>", text, count=1)
    if count != 1:
        raise ValueError(f"expected exactly one {name}[1] assignment, found {count}")
    return updated


def update_solver_file(path, min_rad, max_rad, dry_run):
    original = path.read_text()
    updated = replace_bound(original, "lbu", min_rad)
    updated = replace_bound(updated, "ubu", max_rad)

    changed = updated != original
    if changed and not dry_run:
        path.write_text(updated)
    return changed


def main():
    parser = argparse.ArgumentParser(
        description="Update steering bounds in all generated ACADOS solver C files."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print what would change without writing files",
    )
    args = parser.parse_args()

    acados_dir = Path(__file__).resolve().parent
    solver_files = sorted(acados_dir.glob("c_generated_code*/acados_solver_mobile_robot_*.c"))
    if not solver_files:
        print(f"No generated solver files found under {acados_dir}", file=sys.stderr)
        return 1

    max_right_rad = math.radians(STEERING_MAX_RIGHT_DEG)
    max_left_rad = math.radians(STEERING_MAX_LEFT_DEG)

    print("Steering bounds:")
    print(f"  max right: {STEERING_MAX_RIGHT_DEG:.6f} deg = {format_rad(max_right_rad)} rad")
    print(f"  max left:  {STEERING_MAX_LEFT_DEG:.6f} deg = {format_rad(max_left_rad)} rad")

    changed_count = 0
    for path in solver_files:
        try:
            changed = update_solver_file(path, max_right_rad, max_left_rad, args.dry_run)
        except ValueError as exc:
            print(f"ERROR: {path}: {exc}", file=sys.stderr)
            return 1

        status = "would update" if args.dry_run and changed else "updated" if changed else "unchanged"
        print(f"  {status}: {path.relative_to(acados_dir)}")
        changed_count += int(changed)

    action = "Would update" if args.dry_run else "Updated"
    print(f"{action} {changed_count} of {len(solver_files)} solver files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
