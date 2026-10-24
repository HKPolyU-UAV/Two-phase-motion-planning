#!/usr/bin/env python3
"""Generate the vertical-avoidance Gazebo world from the planner's task YAML.

Defaults refer to this workspace's source packages. Explicit --base-world,
--task-config and --output paths also work without a running ROS installation.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def finite_number(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def text_element(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def indent(element, level=0):
    """Pretty-print on Python 3.8, before ElementTree.indent was available."""
    whitespace = "\n" + "  " * level
    if len(element):
        if not element.text or not element.text.strip():
            element.text = whitespace + "  "
        for child in element:
            indent(child, level + 1)
        element[-1].tail = whitespace
    if level and (not element.tail or not element.tail.strip()):
        element.tail = whitespace


def add_cylinder(world, name, obstacle):
    values = {key: finite_number(obstacle[key], name + "/" + key)
              for key in ("x", "y", "z", "r", "height")}
    if values["r"] <= 0 or values["height"] <= 0:
        raise ValueError("{} radius and height must be positive".format(name))
    model = ET.SubElement(world, "model", name=name)
    text_element(model, "static", "true")
    text_element(model, "pose", "{} {} {} 0 0 0".format(
        values["x"], values["y"], values["z"]))
    link = ET.SubElement(model, "link", name="link")
    text_element(link, "gravity", "false")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=kind)
        cylinder = ET.SubElement(ET.SubElement(element, "geometry"), "cylinder")
        text_element(cylinder, "radius", values["r"])
        text_element(cylinder, "length", values["height"])
        if kind == "visual":
            # Match the blue, half-transparent static markers in RViz.
            material = ET.SubElement(element, "material")
            text_element(material, "ambient", "0 0 1 1")
            text_element(material, "diffuse", "0 0 1 1")
            text_element(element, "transparency", "0.5")


def generate(base_world, task_config, output):
    if output.resolve() == base_world.resolve():
        raise ValueError("Output must differ from the original world")
    with task_config.open() as stream:
        config = yaml.safe_load(stream)
    obstacles = config["static_obstacles"]
    count = int(obstacles["num"])
    if count < 0 or count != obstacles["num"]:
        raise ValueError("static_obstacles/num must be a nonnegative integer")
    pose = config["vertical_validation"]["start_pose"]
    if len(pose) != 6:
        raise ValueError("vertical_validation/start_pose needs six components")
    pose = [finite_number(value, "start_pose") for value in pose]

    tree = ET.parse(str(base_world))
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Base SDF has no world")
    airplane = world.find("model[@name='suav1']")
    ground = world.find("model[@name='ground_plane']")
    if airplane is None or ground is None or airplane.find("plugin") is None:
        raise ValueError("Base world must contain ground_plane and suav1 with its controller")

    # Saved states contain old poses, times and velocities and override model
    # definitions. Remove them so this validation always starts at its own pose.
    for state in world.findall("state"):
        world.remove(state)
    for model in world.findall("model"):
        if model not in (airplane, ground):
            world.remove(model)
    airplane_pose = airplane.find("pose")
    if airplane_pose is None:
        airplane_pose = ET.SubElement(airplane, "pose")
    airplane_pose.text = " ".join(str(value) for value in pose)

    ground_size = config["vertical_validation"].get("ground_size")
    if ground_size is not None:
        if len(ground_size) != 2:
            raise ValueError("vertical_validation/ground_size needs two components")
        ground_size = [finite_number(v, "ground_size") for v in ground_size]
        if min(ground_size) <= 0:
            raise ValueError("ground_size must be positive")
        for size in ground.findall(".//geometry/plane/size"):
            size.text = " ".join(str(v) for v in ground_size)

    for number in range(1, count + 1):
        add_cylinder(world, "static_obs{}".format(number),
                     obstacles["obs{}".format(number)])

    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world, "gui", fullscreen="0")
    camera = gui.find("camera")
    if camera is None:
        camera = ET.SubElement(gui, "camera", name="user_camera")
    camera_pose = camera.find("pose")
    if camera_pose is None:
        camera_pose = ET.SubElement(camera, "pose")
    camera_values = config["vertical_validation"].get(
        "camera_pose", [260, -500, 320, 0, 0.55, math.pi / 2])
    if len(camera_values) != 6:
        raise ValueError("vertical_validation/camera_pose needs six components")
    camera_pose.text = " ".join(str(finite_number(v, "camera_pose")) for v in camera_values)
    for name, value in (("view_controller", "orbit"),
                        ("projection_type", "perspective")):
        field = camera.find(name)
        if field is None:
            field = ET.SubElement(camera, name)
        field.text = value

    indent(tree.getroot())
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output), encoding="utf-8", xml_declaration=True)
    with output.open("a") as stream:
        stream.write("\n")
    print("Generated {} with {} static cylinders".format(output, count))


def main():
    package = Path(__file__).resolve().parent.parent
    world_dir = package.parent / "simple-airplane/simple_airplane_gazebo/worlds"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, default=world_dir / "obs.world")
    parser.add_argument("--task-config", type=Path,
                        default=package / "launch/config/task_3d_validation.yaml")
    parser.add_argument("--output", type=Path,
                        default=world_dir / "vertical_avoidance.world")
    args = parser.parse_args()
    try:
        generate(args.base_world, args.task_config, args.output)
    except (KeyError, TypeError, ValueError, OSError, ET.ParseError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
