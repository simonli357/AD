import pandas as pd
import random

# Load the CSV file
file_path = 'assets/coordinates_with_context.csv'  # Update with your actual CSV file path
data = pd.read_csv(file_path)
filtered_data = data[data['Type'] != 'Intersection']
filtered_data = filtered_data[data['Type'] != 'Lane']
filtered_data = filtered_data[data['Type'] != 'Destination']
sign_options = ["stop_sign/model.sdf", "priority_sign/model.sdf"]

model_mapping = {
    "Sign": "sign/model.sdf",
    "Parking": "parking_sign/model.sdf",
    "Roundabout": "roundabout_sign/model.sdf",
    "Crosswalk": "crosswalk_sign/model.sdf",
    "Highway Entrance": "enter_highway_sign/model.sdf",
    "Highway Exit": "leave_highway_sign/model.sdf",
    "Oneway": "oneway_sign/model.sdf",
    "Car": "rcCar_assembly_obstacle/model.sdf",
    "Light": "traffic_light/model.sdf"
}

def generate_launch_file(data):
    # Start building the launch file
    launch_file = ['<?xml version="1.0" encoding="UTF-8"?>\n',
                   '<launch>\n']
    
    # Loop over the data to create nodes for each object
    for index, row in data.iterrows():
        if row['Type'] == "Sign":
            # Randomly select one of the models for signs
            model_file = random.choice(sign_options)
        else:
            model_file = model_mapping.get(row['Type'], "default_model/model.sdf")
        
        model_name = f"sign_{index}"
        x = row['X']
        y = row['Y']
        orientation = row['Orientation']
        if model_file == "traffic_light/model.sdf":
            orientation = (orientation + 3.14159) % (2 * 3.14159)
        if model_file == "enter_highway_sign/model.sdf" or model_file == "leave_highway_sign/model.sdf":
            orientation = (orientation + 3.14159) % (2 * 3.14159)
        
        #Z value: signs: 0.078998, car: 0.032939, light: 0, pedestrian: 0
        z = 0.078998
        if model_file == "rcCar_assembly_obstacle/model.sdf":
            z = 0.032939
        elif model_file == "traffic_light/model.sdf":
            z = 0
        node = (f'  <node\n'
                f'      name  = "{model_name}"\n'
                f'      pkg   = "gazebo_ros"\n'
                f'      type  = "spawn_model"\n'
                f'      args  = "-file $(find models_pkg)/{model_file} -sdf '
                f'-model {model_name} -x {x} -y {y} -z {z} -R 0 -P 0 -Y {orientation}"/>\n\n')
        
        launch_file.append(node)
    
    # Close the launch file
    launch_file.append('</launch>\n')
    
    return ''.join(launch_file)

# Generate the launch file content
launch_file_content_filtered = generate_launch_file(filtered_data)

# Write the generated launch file content to a file
output_file_path = 'assets/spawn_signs.launch'  # Update with your desired output file path
with open(output_file_path, 'w') as file:
    file.write(launch_file_content_filtered)

print(f"Launch file saved to {output_file_path}")
