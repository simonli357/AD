import os
import pandas as pd
import numpy as np

def fill_dummy_estimates(input_csv, output_csv):
    """
    Reads 'input_csv' which should have a header in the second row.
    The file must include the columns 'measured_lateral' and 'measured_longitudinal'.
    Creates two new columns 'estimated_lateral' and 'estimated_longitudinal' by adding
    small random deviations to the measured values, then saves the result to 'output_csv'.
    """
    # Use header=1 to skip the first metadata row and use the second row as header
    df = pd.read_csv(input_csv, header=1)
    
    # Check for required columns
    required_cols = ["measured_lateral", "measured_longitudinal"]
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"Missing required column '{col}' in {input_csv}.")

    # Generate dummy estimated values
    # Adding small Gaussian noise to simulate estimation errors.
    np.random.seed(42)  # For reproducibility
    df["estimated_lateral"] = df["measured_lateral"] + np.random.normal(loc=0, scale=0.05, size=len(df))
    df["estimated_longitudinal"] = df["measured_longitudinal"] + np.random.normal(loc=0, scale=0.1, size=len(df))

    # Save the updated DataFrame to a new CSV
    df.to_csv(output_csv, index=False)
    print(f"Dummy estimated columns added. Output saved to: {output_csv}")

if __name__ == "__main__":
    # Get the current directory and set file paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    input_path = os.path.join(current_dir, "input.csv")
    output_path = os.path.join(current_dir, "output.csv")
    
    fill_dummy_estimates(input_path, output_path)
