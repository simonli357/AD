import re
import math
from pathlib import Path

# 1) Read the entire contents of constants.h
# with open("constants.h", "r") as f:
with open(Path(__file__).parent / "constants.h", "r") as f:
    text = f.read()

# 2) Use a regular expression to locate the REALSENSE_TF array
#    We look for the exact identifier followed by “{ … }”
pattern = r"REALSENSE_TF\s*=\s*\{([^}]+)\}"
match = re.search(pattern, text)
if not match:
    raise RuntimeError("Could not find REALSENSE_TF in constants.h")

# 3) The first (and only) capturing group is everything between the braces.
values_str = match.group(1).strip()  # e.g. "-0.08255 - 0.0042, -0.032, 0.257, 0, 1.85*M_PI/180.0, 2.1*M_PI/180.0"

# 4) Split on commas, then clean up whitespace
tokens = [tok.strip() for tok in values_str.split(",")]

# 5) For each token, replace “M_PI” with “math.pi” so Python can evaluate it.
def sanitize(expr: str) -> str:
    return expr.replace("M_PI", "math.pi")

# 6) Evaluate each token
numbers = []
for tok in tokens:
    expr = sanitize(tok)
    try:
        val = eval(expr, {"math": math})
    except Exception as e:
        raise RuntimeError(f"Failed to evaluate '{expr}': {e}")
    numbers.append(float(val))

if len(numbers) != 6:
    raise RuntimeError(f"Expected 6 values, but found {len(numbers)}: {numbers}")

# 7) Unpack into named variables
x, y, z, roll, pitch, yaw = numbers

# 8) Now you can use these six variables however you like.
print("REALSENSE extrinsics (x, y, z, roll, pitch, yaw):")
print(f"  x     = {x:.6f}")
print(f"  y     = {y:.6f}")
print(f"  z     = {z:.6f}")
print(f"  roll  = {roll:.6f}  (radians)")
print(f"  pitch = {pitch:.6f}  (radians)")
print(f"  yaw   = {yaw:.6f}  (radians)")