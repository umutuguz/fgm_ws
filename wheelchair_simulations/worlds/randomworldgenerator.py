import random
import xml.etree.ElementTree as ET

# Load the original .world file
tree = ET.parse('montecarloworld_1.world')
root = tree.getroot()

# Define the limits for randomization
x_range1 = [-26, 4]
x_range2 = [-4, -18]
y_range = [-3, 25]

# Define the number of variations to generate
num_variations = 500

# Generate and save multiple variations
for i in range(num_variations):
    # Create a copy of the original root element for each variation
    variation_root = ET.Element(root.tag)

    # Copy the attributes of the original root element
    variation_root.attrib = root.attrib

    # Randomize the x and y values for each box
    for model in root.findall(".//model"):
        model_name = model.attrib.get('name')
        if model_name.startswith('box'):
            x = random.uniform(*x_range1) if int(model_name[3:]) in [1, 4, 6, 7] else random.uniform(*x_range2)
            y = random.uniform(*y_range)
            pose_element = model.find('pose')
            pose_element.text = f'{x} {y} 1.25 0 -0 0'
        variation_root.append(model)

    # Randomize the x and y values for each cylinder
    for model in root.findall(".//model"):
        model_name = model.attrib.get('name')
        if model_name.startswith('cylinder'):
            x = random.uniform(*x_range1) if int(model_name[8:]) in [1, 2, 3, 8] else random.uniform(*x_range2)
            y = random.uniform(*y_range)
            pose_element = model.find('pose')
            pose_element.text = f'{x} {y} 1.25 0 -0 0'
        variation_root.append(model)

    # Save the modified .world file
    variation_file_name = f'variation_{i+1}.world'
    variation_tree = ET.ElementTree(variation_root)
    variation_tree.write(variation_file_name)

    print(f'Saved {variation_file_name}')

print('All variations saved!')
