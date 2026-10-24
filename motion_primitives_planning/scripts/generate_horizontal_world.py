#!/usr/bin/env python3
"""Replace the floating-sphere experiment with fixed horizontal cylinders.

This derivative keeps the original twelve upright cylinder geometries and the
flight controller, and adds any further upright cylinders from the task config.
Every fixed obstacle is blue; dynamic cylinders remain red and are supplied by
the existing Gazebo marker bridge.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


# The original Gazebo world uses editor-generated names for obstacles 2--12.
# Keep their saved geometry and poses, including the small differences from the
# conservative planning configuration, instead of creating duplicate cylinders.
ORIGINAL_STATIC_MODELS = {
    1: "static_obs1",
    2: "unit_cylinder_clone",
    3: "unit_cylinder_clone_0",
    4: "unit_cylinder_clone_1",
    5: "unit_cylinder_clone_2",
    6: "unit_cylinder_clone_3_clone_0_clone",
    7: "unit_cylinder_clone_3_clone",
    8: "unit_cylinder_clone_3_clone_0_clone_0_clone",
    9: "unit_cylinder_clone_3_clone_0_clone_0",
    10: "unit_cylinder_clone_3_clone_0_clone_0_clone_clone",
    11: "unit_cylinder_clone_3_clone_0",
    12: "unit_cylinder_clone_3_clone_0_clone_1",
}


def finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(name + " must be finite")
    return result


def field(parent, name, value):
    result = ET.SubElement(parent, name)
    result.text = str(value)
    return result


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


def paint_blue(model):
    for visual in model.findall("link/visual"):
        for material in visual.findall("material"):
            visual.remove(material)
        material = ET.SubElement(visual, "material")
        field(material, "ambient", "0 0 1 1")
        field(material, "diffuse", "0 0 1 1")
        transparency = visual.find("transparency")
        if transparency is None:
            transparency = ET.SubElement(visual, "transparency")
        transparency.text = "0.5"


def add_horizontal(world, name, config):
    c = {key: finite(config[key], name + "/" + key)
         for key in ("x", "y", "z", "r", "length", "axis_yaw")}
    if min(c["r"], c["length"]) <= 0:
        raise ValueError(name + " dimensions must be positive")
    if world.find("model[@name='" + name + "']") is not None:
        raise ValueError("Base world already has model " + name)
    model = ET.SubElement(world, "model", name=name)
    field(model, "static", "true")
    # SDF cylinders point along local Z; Ry(pi/2), then Rz(yaw), makes the
    # world-space axis (cos(yaw), sin(yaw), 0), matching the planner and RViz.
    field(model, "pose", "{} {} {} 0 {} {}".format(
        c["x"], c["y"], c["z"], math.pi / 2, math.radians(c["axis_yaw"])))
    link = ET.SubElement(model, "link", name="link")
    field(link, "gravity", "false")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=kind)
        cylinder = ET.SubElement(ET.SubElement(element, "geometry"), "cylinder")
        field(cylinder, "radius", c["r"])
        field(cylinder, "length", c["length"])
    paint_blue(model)


def add_upright(world, name, config, default_height):
    c = {key: finite(config[key], name + "/" + key) for key in ("x", "y", "r")}
    c["height"] = finite(config.get("height", default_height), name + "/height")
    c["z"] = finite(config.get("z", c["height"] / 2), name + "/z")
    if min(c["r"], c["height"]) <= 0:
        raise ValueError(name + " dimensions must be positive")
    if world.find("model[@name='" + name + "']") is not None:
        raise ValueError("Base world already has model " + name)
    model = ET.SubElement(world, "model", name=name)
    field(model, "static", "true")
    field(model, "pose", "{} {} {} 0 0 0".format(c["x"], c["y"], c["z"]))
    link = ET.SubElement(model, "link", name="link")
    field(link, "gravity", "false")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=kind)
        cylinder = ET.SubElement(ET.SubElement(element, "geometry"), "cylinder")
        field(cylinder, "radius", c["r"])
        field(cylinder, "length", c["height"])
    paint_blue(model)


def generate(base_world, task_config, output):
    if base_world.resolve() == output.resolve():
        raise ValueError("Output must differ from the original world")
    with task_config.open() as stream:
        config = yaml.safe_load(stream)
    obstacles = config["horizontal_obstacles"]
    count = int(obstacles["num"])
    if count < 1 or count != obstacles["num"]:
        raise ValueError("horizontal_obstacles/num must be a positive integer")
    tree = ET.parse(str(base_world))
    world = tree.getroot().find("world")
    if world is None or world.find("model[@name='suav1']/plugin") is None:
        raise ValueError("Base world must contain suav1 and its flight controller")
    for state in world.findall("state"):
        world.remove(state)
    for model in world.findall("model"):
        if model.get("name", "").startswith("sphere_obs"):
            world.remove(model)
        elif model.find("link/collision/geometry/cylinder") is not None:
            paint_blue(model)
    static = config["static_obstacles"]
    static_count = int(static["num"])
    if static_count < 0 or static_count != static["num"]:
        raise ValueError("static_obstacles/num must be a nonnegative integer")
    default_height = static.get("height", 36.2743)
    added_upright = 0
    for i in range(1, static_count + 1):
        name = "static_obs" + str(i)
        original_name = ORIGINAL_STATIC_MODELS.get(i, name)
        if (world.find("model[@name='" + original_name + "']") is None and
                world.find("model[@name='" + name + "']") is None):
            add_upright(world, name, static["obs" + str(i)], default_height)
            added_upright += 1
    for i in range(1, count + 1):
        add_horizontal(world, "horizontal_obs" + str(i), obstacles["cylinder" + str(i)])

    display = config.get("horizontal_validation", {})
    size = display.get("ground_size")
    if size is not None:
        if len(size) != 2:
            raise ValueError("ground_size needs two components")
        size = [finite(v, "ground_size") for v in size]
        if min(size) <= 0:
            raise ValueError("ground_size must be positive")
        for element in world.findall("model[@name='ground_plane']//geometry/plane/size"):
            element.text = " ".join(str(v) for v in size)
    pose = display.get("camera_pose")
    if pose is not None:
        if len(pose) != 6:
            raise ValueError("camera_pose needs six components")
        pose = [finite(v, "camera_pose") for v in pose]
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
    print("Generated {} with {} fixed horizontal cylinders and {} added upright cylinders".format(
        output, count, added_upright))


def main():
    package = Path(__file__).resolve().parent.parent
    worlds = package.parent / "simple-airplane/simple_airplane_gazebo/worlds"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, default=worlds / "obs.world")
    parser.add_argument("--task-config", type=Path,
                        default=package / "launch/config/task_3d_horizontal.yaml")
    parser.add_argument("--output", type=Path, default=worlds / "horizontal_obstacles.world")
    args = parser.parse_args()
    try:
        generate(args.base_world, args.task_config, args.output)
    except (KeyError, TypeError, ValueError, OSError, ET.ParseError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
