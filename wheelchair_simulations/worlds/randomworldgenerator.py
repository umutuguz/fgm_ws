import xml.etree.ElementTree as ET
import random

tree = ET.parse('ornekworld.world')
root = tree.getroot()

def find_collision_box_size(model_element):
    if model_element is not None:
        linkk_element = model_element.find('link')
        if linkk_element is not None:
            collision_element = linkk_element.find('collision')
        if collision_element is not None:
            geometry_element = collision_element.find('geometry')
            if geometry_element is not None:
                box_element = geometry_element.find('box')
                if box_element is not None:
                    size_element = box_element.find('size')
                    if size_element is not None:
                        return size_element
    
    return None  # Any step returned None or 'radius' not found

def find_visual_box_size(model_element):
    if model_element is not None:
        linkk_element = model_element.find('link')
        if linkk_element is not None:
            visual_element = linkk_element.find('visual')
        if visual_element is not None:
            geometry_element = visual_element.find('geometry')
            if geometry_element is not None:
                box_element = geometry_element.find('box')
                if box_element is not None:
                    size_element = box_element.find('size')
                    if size_element is not None:
                        return size_element
    
    return None  # Any step returned None or 'radius' not found

def find_collision_radius(model_element):
    if model_element is not None:
        linkk_element = model_element.find('link')
        if linkk_element is not None:
            collision_element = linkk_element.find('collision')
        if collision_element is not None:
            geometry_element = collision_element.find('geometry')
            if geometry_element is not None:
                cylinder_element = geometry_element.find('cylinder')
                if cylinder_element is not None:
                    radius_element = cylinder_element.find('radius')
                    if radius_element is not None:
                        return radius_element
    
    return None  # Any step returned None or 'radius' not found

def find_visual_radius(model_element):
    if model_element is not None:
        linkk_element = model_element.find('link')
        if linkk_element is not None:
            visual_element = linkk_element.find('visual')
        if visual_element is not None:
            geometry_element = visual_element.find('geometry')
            if geometry_element is not None:
                cylinder_element = geometry_element.find('cylinder')
                if cylinder_element is not None:
                    radius_element = cylinder_element.find('radius')
                    if radius_element is not None:
                        return radius_element
    
    return None  # Any step returned None or 'radius' not found

for i in range(1,5):
    for model_element in root.findall('.//model'):
        name = model_element.get('name')
        if name.startswith('box') or name.startswith('cylinder'):
            randy = random.uniform(-3, 25)
            if (randy<4 or randy > 18):
                randx = random.uniform(-18, -4)
            else:
                randx = random.uniform(-26, 4)

            rand_radius = random.uniform(0.3, 0.7)

            rand_size_x = random.uniform(0.5, 2.1)
            if rand_size_x > 1.3:
                rand_size_y = random.uniform(0.5, 0.9)
            elif rand_size_x < 0.9:
                rand_size_y = random.uniform(1.3, 2.1)
            else:
                rand_size_y = random.uniform(0.9, 1.3)

            pose_element = model_element.find('pose')

            link_element = model_element.find('link')

            link_pose = link_element.find('pose')

            collision_radius_element = find_collision_radius(model_element)
            visual_radius_element = find_visual_radius(model_element)

            box_visual_size_element = find_visual_box_size(model_element)
            box_collision_size_element = find_collision_box_size(model_element)

            if pose_element is not None:
                pose_element.text = f"{randx} {randy} 1.25 0 0 0"
            if link_pose is not None:
                link_pose.text = f"{randx} {randy} 1.25 0 0 0"
            if collision_radius_element is not None:
                collision_radius_element.text = f"{rand_radius}"
            if visual_radius_element is not None:
                visual_radius_element.text = f"{rand_radius}"
            if box_visual_size_element is not None:
                box_visual_size_element.text = f"{rand_size_x} {rand_size_y} 2.5"
            if box_collision_size_element is not None:
                box_collision_size_element.text = f"{rand_size_x} {rand_size_y} 2.5"

    tree.write(f"montecarloworld_{i+1}.world")