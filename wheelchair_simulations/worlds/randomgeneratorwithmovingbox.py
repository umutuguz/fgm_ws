import random
import xml.etree.ElementTree as ET
import os

def random_value(min_val, max_val):
    return random.uniform(min_val, max_val)

def modify_world_file(file_path, output_path, index):
    tree = ET.parse(file_path)
    root = tree.getroot()

    for model in root.findall(".//model"):
        model_name = model.get('name')
        if model_name and model_name.startswith('moving_box_'):
            # Randomize pose
            pose = model.find('pose')
            x = random_value(3, 19)  # X position between 3 and 19
            y = random_value(-3, 3)
            pose.text = f"{x} {y} 0 0 0 0"

            # Randomize size
            size_x = random_value(0.3, 1.2)
            size_y = random_value(0.3, 1.2)
            size_text = f"{size_x} {size_y} 1"
            for element in model.findall('.//collision/geometry/box/size') + model.findall('.//visual/geometry/box/size'):
                element.text = size_text

            # Set color to red
            for visual in model.findall('.//visual'):
                material = visual.find('material')
                if material is None:
                    material = ET.SubElement(visual, 'material')
                script = material.find('script')
                if script is None:
                    script = ET.SubElement(material, 'script')
                script.clear()  # Remove existing script elements if any
                ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'
                ET.SubElement(script, 'name').text = 'Gazebo/Red'

            # Randomize velocity
            xvel = random_value(-0.18, 0.18)
            if y < 0:
                yvel = random_value(0.07, 0.22)
            else:
                yvel = random_value(-0.22, -0.07)

            velocity = model.find(".//plugin/velocity")
            if velocity is not None:
                velocity.text = f"{xvel} {yvel} 0"

    # Save the modified world file
    new_world_file = os.path.join(output_path, f'montecarloworld_modified_with_moving_boxes_{index}.world')
    tree.write(new_world_file)

# Path to the original world file and output directory
worlds_directory = os.path.expanduser("~/fgm_ws/src/wheelchair_simulations/worlds")
original_world_file = os.path.join(worlds_directory, 'montecarloworld_modified_with_moving_boxes.world')
output_directory = worlds_directory

# Generate 300 modified world files
for i in range(1, 301):
    modify_world_file(original_world_file, output_directory, i)
    print(f"Generated world file {i}")

