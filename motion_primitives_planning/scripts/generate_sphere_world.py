#!/usr/bin/env python3
"""Add fixed floating spheres to the original twelve-column Gazebo world.

The existing column models and the aircraft are copied without changing their
poses or dimensions. Dynamic cylinders continue to come from the marker bridge.
Regenerate this derivative world after editing sphere_obstacles in its task YAML.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def finite_number(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(name + " must be finite")
    return value


def field(parent, name, value):
    element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def indent(element, level=0):
    space = "\n" + "  " * level
    if len(element):
        if not element.text or not element.text.strip():
            element.text = space + "  "
        for child in element:
            indent(child, level + 1)
        element[-1].tail = space
    if level and (not element.tail or not element.tail.strip()):
        element.tail = space


def add_sphere(world, name, config):
    values = {key: finite_number(config[key], name + "/" + key)
              for key in ("x", "y", "z", "r")}
    if values["r"] <= 0:
        raise ValueError(name + " radius must be positive")
    if world.find("model[@name='" + name + "']") is not None:
        raise ValueError("Base world already has model " + name)
    model = ET.SubElement(world, "model", name=name)
    field(model, "static", "true")
    field(model, "pose", "{} {} {} 0 0 0".format(values["x"], values["y"], values["z"]))
    link = ET.SubElement(model, "link", name="link")
    field(link, "gravity", "false")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=kind)
        sphere = ET.SubElement(ET.SubElement(element, "geometry"), "sphere")
        field(sphere, "radius", values["r"])
        if kind == "visual":
            material = ET.SubElement(element, "material")
            field(material, "ambient", "0 0 1 1")
            field(material, "diffuse", "0 0 1 1")
            field(element, "transparency", "0.5")


def generate(base_world, task_config, output):
    if base_world.resolve() == output.resolve():
        raise ValueError("Output must differ from the original world")
    with task_config.open() as stream:
        config = yaml.safe_load(stream)
    spheres = config["sphere_obstacles"]
    count = int(spheres["num"])
    if count < 1 or count != spheres["num"]:
        raise ValueError("sphere_obstacles/num must be a positive integer")
    tree = ET.parse(str(base_world))
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Base SDF has no world")
    if world.find("model[@name='suav1']/plugin") is None:
        raise ValueError("Base world must contain suav1 and its controller")
    # Root model poses in obs.world already agree with the saved state. Removing
    # the state resets time to zero and avoids stale state overriding new models.
    for state in world.findall("state"):
        world.remove(state)
    for i in range(1, count + 1):
        add_sphere(world, "sphere_obs" + str(i), spheres["sphere" + str(i)])

    display = config.get("sphere_validation", {})
    size = display.get("ground_size")
    if size is not None:
        if len(size) != 2:
            raise ValueError("ground_size needs two components")
        size = [finite_number(v, "ground_size") for v in size]
        if min(size) <= 0:
            raise ValueError("ground_size must be positive")
        for element in world.findall("model[@name='ground_plane']//geometry/plane/size"):
            element.text = " ".join(str(v) for v in size)
    pose = display.get("camera_pose")
    if pose is not None:
        if len(pose) != 6:
            raise ValueError("camera_pose needs six components")
        pose = [finite_number(v, "camera_pose") for v in pose]
        gui = world.find("gui")
        if gui is None:
            gui = ET.SubElement(world, "gui", fullscreen="0")
        camera = gui.find("camera")
        if camera is None:
            camera = ET.SubElement(gui, "camera", name="user_camera")
        for name, value in (("pose", " ".join(str(v) for v in pose)),
                            ("view_controller", "orbit"),
                            ("projection_type", "perspective")):
            element = camera.find(name)
            if element is None:
                element = ET.SubElement(camera, name)
            element.text = value
    indent(tree.getroot())
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output), encoding="utf-8", xml_declaration=True)
    with output.open("a") as stream:
        stream.write("\n")
    print("Generated {} with {} added floating spheres".format(output, count))


def main():
    package = Path(__file__).resolve().parent.parent
    worlds = package.parent / "simple-airplane/simple_airplane_gazebo/worlds"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, default=worlds / "obs.world")
    parser.add_argument("--task-config", type=Path,
                        default=package / "launch/config/task_3d_spheres.yaml")
    parser.add_argument("--output", type=Path, default=worlds / "sphere_obstacles.world")
    args = parser.parse_args()
    try:
        generate(args.base_world, args.task_config, args.output)
    except (KeyError, TypeError, ValueError, OSError, ET.ParseError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
