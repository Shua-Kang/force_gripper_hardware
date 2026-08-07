"""Generate a combined URDF: Franka Panda arm + the force-control gripper.

Takes pybullet_data's franka_panda/panda.urdf, removes the stock Franka hand
and fingers, and grafts the force gripper onto panda_link8 with a fixed mount
joint (plus a massless `grasp_target` frame at the pinch center, which
panda-gym uses as the end-effector link).

Franka mesh paths are rewritten to absolute paths into pybullet_data, so the
output is machine-specific: re-run this script after moving the repo or the
venv. Output: urdf/panda_with_force_gripper.urdf

    uv run python scripts/make_panda_with_gripper.py
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pybullet_data

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "urdf"
GRIPPER_URDF = URDF_DIR / "force_control_gripper.urdf"
OUT_URDF = URDF_DIR / "panda_with_force_gripper.urdf"

PANDA_DIR = Path(pybullet_data.getDataPath()) / "franka_panda"

# Links/joints of the stock Franka hand to strip out.
DROP_LINKS = {"panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_grasptarget"}
DROP_JOINTS = {"panda_hand_joint", "panda_finger_joint1", "panda_finger_joint2", "panda_grasptarget_hand"}

# Gripper base mount: the back face of base-1 (local y = -65 mm) sits on the
# panda_link8 flange, fingers pointing along the tool +Z axis, with the same
# -45 deg twist the stock hand uses.
MOUNT_XYZ = "0 0 0.065"
MOUNT_RPY = f"{math.pi / 2} 0 {-math.pi / 4}"

# Pinch center in the base-1 frame (fingertip pads are around y = 0.13 m).
TCP_XYZ = "0 0.13 0"


def main() -> None:
    panda = ET.parse(PANDA_DIR / "panda.urdf").getroot()
    panda.set("name", "panda_with_force_gripper")

    # Drop the stock hand.
    for el in list(panda):
        name = el.get("name")
        if (el.tag == "link" and name in DROP_LINKS) or (el.tag == "joint" and name in DROP_JOINTS):
            panda.remove(el)

    # Franka meshes: "package://meshes/..." -> absolute path into pybullet_data.
    for mesh in panda.iter("mesh"):
        fn = mesh.get("filename")
        if fn.startswith("package://"):
            mesh.set("filename", str(PANDA_DIR / fn[len("package://"):]))

    # Graft the gripper links/joints (mesh paths stay relative to URDF_DIR).
    gripper = ET.parse(GRIPPER_URDF).getroot()
    for el in gripper:
        panda.append(el)

    mount = ET.SubElement(panda, "joint", name="gripper_mount", type="fixed")
    ET.SubElement(mount, "origin", xyz=MOUNT_XYZ, rpy=MOUNT_RPY)
    ET.SubElement(mount, "parent", link="panda_link8")
    ET.SubElement(mount, "child", link="base-1")

    # Massless TCP frame at the pinch center (panda-gym's ee_link).
    tcp_link = ET.SubElement(panda, "link", name="grasp_target")
    inertial = ET.SubElement(tcp_link, "inertial")
    ET.SubElement(inertial, "mass", value="0.0")
    ET.SubElement(inertial, "inertia", ixx="0", iyy="0", izz="0", ixy="0", iyz="0", ixz="0")
    tcp = ET.SubElement(panda, "joint", name="grasp_target_joint", type="fixed")
    ET.SubElement(tcp, "origin", xyz=TCP_XYZ, rpy="0 0 0")
    ET.SubElement(tcp, "parent", link="base-1")
    ET.SubElement(tcp, "child", link="grasp_target")

    ET.indent(ET.ElementTree(panda), space="  ")
    OUT_URDF.write_bytes(ET.tostring(panda, xml_declaration=True, encoding="utf-8"))
    print(f"wrote {OUT_URDF}")

    # Print the joint table of the combined model for reference.
    import pybullet as p

    p.connect(p.DIRECT)
    rid = p.loadURDF(str(OUT_URDF), useFixedBase=True)
    for i in range(p.getNumJoints(rid)):
        info = p.getJointInfo(rid, i)
        jtype = {0: "rev", 1: "prism", 4: "fixed"}.get(info[2], str(info[2]))
        print(f"  joint {i:2d}  {info[1].decode():24s} {jtype:6s} child link: {info[12].decode()}")
    p.disconnect()


if __name__ == "__main__":
    main()
