#!/usr/bin/env python3
"""Print the torque limits and the total mass written in a robot model file.

Usage:
    python3 check_limits.py MODEL.urdf [JOINT_SUBSTRING]
    python3 check_limits.py MODEL.xml  [JOINT_SUBSTRING]

URDF: each joint's <limit effort="...">, and the sum of every link's
<inertial><mass value="...">. Links with no mass are listed.
MJCF (MuJoCo): each joint's actuatorfrcrange, each actuator's forcerange, and
for <motor> actuators ctrlrange x gear; the sum of every body's
<inertial mass="...">. Bodies with no <inertial> are listed, because MuJoCo
then infers their mass from their geoms, so the true total is higher.

Limits: values set only through MJCF <default> classes are not resolved, and
<include>d files are not followed. Standard library only.
"""
import sys
import xml.etree.ElementTree as ET


def _range_max(text):
    vals = [abs(float(v)) for v in text.split()]
    return max(vals) if vals else None


def read_urdf(root):
    rows = []
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if limit is None or limit.get("effort") is None:
            continue
        rows.append((joint.get("name"), "limit effort", float(limit.get("effort"))))
    mass, no_mass = 0.0, []
    for link in root.iter("link"):
        m = link.find("inertial/mass")
        if m is not None and m.get("value") is not None:
            mass += float(m.get("value"))
        else:
            no_mass.append(link.get("name"))
    return rows, mass, no_mass


def read_mjcf(root):
    rows = []
    for joint in root.iter("joint"):
        frc = joint.get("actuatorfrcrange")
        if frc:
            rows.append((joint.get("name") or "(default class)", "actuatorfrcrange", _range_max(frc)))
    for actuator in root.iter("actuator"):
        for act in actuator:
            name = act.get("name") or act.get("joint") or "(unnamed)"
            if act.get("forcerange"):
                rows.append((name, act.tag + " forcerange", _range_max(act.get("forcerange"))))
            if act.tag == "motor" and act.get("ctrlrange"):
                gear = float((act.get("gear") or "1").split()[0])
                ctrl = _range_max(act.get("ctrlrange"))
                if ctrl is not None:
                    rows.append((name, "motor ctrlrange x gear", ctrl * abs(gear)))
    mass, no_mass = 0.0, []
    for body in root.iter("body"):
        inertial = body.find("inertial")
        if inertial is not None and inertial.get("mass") is not None:
            mass += float(inertial.get("mass"))
        else:
            no_mass.append(body.get("name"))
    return rows, mass, no_mass


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    root = ET.parse(argv[1]).getroot()
    needle = argv[2].lower() if len(argv) > 2 else ""
    if root.tag == "robot":
        kind, (rows, mass, no_mass) = "URDF", read_urdf(root)
    elif root.tag == "mujoco":
        kind, (rows, mass, no_mass) = "MJCF", read_mjcf(root)
    else:
        print("not a URDF (<robot>) or MJCF (<mujoco>) file: root is <%s>" % root.tag)
        return 1
    print("%s %s" % (kind, argv[1]))
    for name, source, value in rows:
        if value is not None and needle in (name or "").lower():
            print("  %-32s %-24s %g N.m" % (name, source, value))
    print("total explicit mass: %.3f kg" % mass)
    if no_mass:
        print("no explicit mass (%d): %s" % (len(no_mass), ", ".join(str(n) for n in no_mass)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
