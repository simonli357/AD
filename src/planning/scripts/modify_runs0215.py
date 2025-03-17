#!/usr/bin/env python3
import yaml
import os

# Define the sequences for modification.
INSERT_SEQ = [419, 520, 372, 328, 259, 314, 350, 435]
REPLACE_SEQ = [240, 280]

def modify_run(run_list):
    """
    Modifies a list of numbers from a run according to the following rules:
      - Finds the first occurrence of 158 or 179 and inserts INSERT_SEQ immediately after.
      - Finds the first occurrence of 196 and replaces that number with REPLACE_SEQ.
    """
    # Determine the index for the first occurrence of 158 or 179.
    insert_idx = None
    for i, num in enumerate(run_list):
        if num in (158, 179):
            insert_idx = i
            break

    # Determine the index for the first occurrence of 196.
    replace_idx = None
    for i, num in enumerate(run_list):
        if num == 201:
            replace_idx = i
            break

    # Prepare a list of modifications as tuples: (mod_type, index).
    # We'll apply modifications in descending order of index to avoid shifting issues.
    mods = []
    if insert_idx is not None:
        mods.append(('insert', insert_idx))
        print(f"Inserting sequence at index {insert_idx}")
    if replace_idx is not None:
        mods.append(('replace', replace_idx))
        print(f"Replacing number at index {replace_idx}")
    mods.sort(key=lambda x: x[1], reverse=True)

    # Apply modifications.
    for mod, idx in mods:
        if mod == 'insert':
            # Insert the sequence AFTER the found element (i.e. at position idx+1).
            run_list[idx+1:idx+1] = INSERT_SEQ
        elif mod == 'replace':
            # Replace the element at idx with the replacement sequence.
            if idx == 0:
              run_list = REPLACE_SEQ + run_list[1:]
            else:
              run_list[idx:idx+1] = REPLACE_SEQ

    return run_list

def main():
    # Load the original YAML file.
    current_dir = os.path.dirname(os.path.realpath(__file__))
    input_file = os.path.join(current_dir, "config/runs0215.yaml")
    with open(input_file, "r") as f:
        runs = yaml.safe_load(f)

    # Process each run.
    i = 0
    for run_name, numbers in runs.items():
        # Modify the run list in place.
        print(f"Processing run {i}: {run_name}")
        i += 1
        runs[run_name] = modify_run(numbers)

    # Write the modified runs to a new YAML file.
    output_file = os.path.join(current_dir, "config/runs0215_modified.yaml")
    with open(output_file, "w") as f:
        yaml.dump(runs, f, default_flow_style=False)
    print(f"Modified runs saved to {output_file}")

if __name__ == "__main__":
    main()
