import sys
import xml.etree.ElementTree as ET

def scale_floats(text, factor):
    vals = [float(v) for v in text.split()]
    vals = [v * factor for v in vals]
    return " ".join(f"{v:.6f}".rstrip("0").rstrip(".") for v in vals)

def scale_pose(pose_text, factor):
    vals = [float(v) for v in pose_text.split()]
    if len(vals) >= 3:
        vals[0] *= factor
        vals[1] *= factor
        vals[2] *= factor
    return " ".join(f"{v:.6f}".rstrip("0").rstrip(".") for v in vals)

def scale_world(root, factor, exclude_models=None):

    if exclude_models is None:
        exclude_models = set()

    for model in root.findall(".//model"):

        name = model.get("name", "")

        # Skip excluded models (e.g. robot)
        if name in exclude_models:
            continue

        # Scale all poses inside this model
        pose = model.find("pose")
        if pose is not None and pose.text:
            pose.text = scale_pose(pose.text, factor)

        # Scale mesh scales
        for scale in model.findall(".//scale"):
            if scale.text:
                scale.text = scale_floats(scale.text, factor)

        # Scale boxes
        for size in model.findall(".//box/size"):
            if size.text:
                size.text = scale_floats(size.text, factor)

        # Scale cylinders
        for radius in model.findall(".//cylinder/radius"):
            radius.text = str(float(radius.text) * factor)

        for length in model.findall(".//cylinder/length"):
            length.text = str(float(length.text) * factor)

        # Scale spheres
        for radius in model.findall(".//sphere/radius"):
            radius.text = str(float(radius.text) * factor)

    # Scale model/world poses
    for pose in root.findall(".//pose"):
        if pose.text:
            pose.text = scale_pose(pose.text, factor)

    # Scale mesh scales
    for scale in root.findall(".//scale"):
        if scale.text:
            scale.text = scale_floats(scale.text, factor)

    # Scale boxes
    for size in root.findall(".//box/size"):
        if size.text:
            size.text = scale_floats(size.text, factor)

    # Scale cylinders
    for radius in root.findall(".//cylinder/radius"):
        radius.text = str(float(radius.text) * factor)

    for length in root.findall(".//cylinder/length"):
        length.text = str(float(length.text) * factor)

    # Scale spheres
    for radius in root.findall(".//sphere/radius"):
        radius.text = str(float(radius.text) * factor)

if __name__ == "__main__":

    if len(sys.argv) != 4:
        print("Usage:")
        print("python scale_world.py input.world output.world scale_factor")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    factor = float(sys.argv[3])

    tree = ET.parse(input_file)
    root = tree.getroot()

    scale_world(
        root,
        factor,
        exclude_models={
            "OpenRobotics/_Rosbot1",
        }
    )

    tree.write(output_file, encoding="utf-8", xml_declaration=True)

    print(f"Scaled world written to {output_file}")