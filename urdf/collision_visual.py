"""Overlay the URDF collision meshes on top of the visual meshes in PyBullet.

PyBullet only renders visual geometry; this module parses the URDF, spawns a
massless, collision-free "ghost" body for every <collision> mesh (drawn in
green), and keeps the ghosts glued to their links every simulation step.

Usage in a demo script:

    vis = CollisionVisualizer(robot_id, URDF_PATH)
    ...
    while running:
        keys = p.getKeyboardEvents()
        vis.handle_keys(keys)     # V cycles: both -> visual only -> collision only
        p.stepSimulation()
        vis.sync()
"""

import os
import xml.etree.ElementTree as ET

import pybullet as p

GHOST_RGBA = (0.15, 0.85, 0.35, 0.55)   # translucent green in overlay mode
GHOST_SOLID = (0.15, 0.85, 0.35, 0.95)  # near-opaque in collision-only mode


class CollisionVisualizer:
    MODES = ("visual + collision", "visual only", "collision only")

    def __init__(self, robot_id, urdf_path):
        self.robot = robot_id
        self.mode = 0

        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

        # link name -> link index (-1 is the base link)
        base_name = p.getBodyInfo(robot_id)[0].decode("utf-8")
        link_index = {base_name: -1}
        for j in range(p.getNumJoints(robot_id)):
            link_index[p.getJointInfo(robot_id, j)[12].decode("utf-8")] = j

        # getBasePositionAndOrientation returns the base link's INERTIAL frame,
        # not its URDF link frame; store the inverse inertial offset so sync()
        # can recover the link frame the <collision> origins are relative to.
        inertial_pos, inertial_orn = p.getDynamicsInfo(robot_id, -1)[3:5]
        self.base_inv_inertial = p.invertTransform(inertial_pos, inertial_orn)

        # Remember original visual colors so we can hide/restore the robot.
        # getVisualShapeData rows: (bodyId, linkIndex, geomType, dims, file, pos, orn, rgba)
        self.visual_rgba = [(row[1], row[7]) for row in p.getVisualShapeData(robot_id)]

        # ghost list: (link index, local pos, local orn, ghost body id)
        self.ghosts = []
        for link in ET.parse(urdf_path).getroot().iter("link"):
            name = link.get("name")
            if name not in link_index:
                continue
            for col in link.findall("collision"):
                mesh = col.find("geometry/mesh")
                if mesh is None:
                    continue
                filename = os.path.join(urdf_dir, mesh.get("filename"))
                scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
                origin = col.find("origin")
                xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
                rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
                vs = p.createVisualShape(
                    p.GEOM_MESH, fileName=filename, meshScale=scale, rgbaColor=GHOST_RGBA
                )
                ghost = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs)
                self.ghosts.append(
                    (link_index[name], xyz, p.getQuaternionFromEuler(rpy), ghost)
                )
        self.sync()

    # ------------------------------------------------------------------ modes
    def handle_keys(self, keys):
        if ord("v") in keys and keys[ord("v")] & p.KEY_WAS_TRIGGERED:
            self.mode = (self.mode + 1) % len(self.MODES)
            self._apply_mode()
            print(f"display mode: {self.MODES[self.mode]}")

    def _apply_mode(self):
        show_visual = self.mode in (0, 1)
        show_ghost = self.mode in (0, 2)
        for link, rgba in self.visual_rgba:
            color = rgba if show_visual else (rgba[0], rgba[1], rgba[2], 0.0)
            p.changeVisualShape(self.robot, link, rgbaColor=color)
        ghost_rgba = GHOST_SOLID if self.mode == 2 else GHOST_RGBA
        for _, _, _, ghost in self.ghosts:
            color = ghost_rgba if show_ghost else (0, 0, 0, 0)
            p.changeVisualShape(ghost, -1, rgbaColor=color)

    # ------------------------------------------------------------------ pose
    def sync(self):
        """Keep the ghost bodies glued to their links. Call after stepSimulation."""
        for link, xyz, orn, ghost in self.ghosts:
            if link == -1:
                com_pos, com_orn = p.getBasePositionAndOrientation(self.robot)
                link_pos, link_orn = p.multiplyTransforms(
                    com_pos, com_orn, *self.base_inv_inertial
                )
            else:
                state = p.getLinkState(self.robot, link, computeForwardKinematics=1)
                link_pos, link_orn = state[4], state[5]
            pos, quat = p.multiplyTransforms(link_pos, link_orn, xyz, orn)
            p.resetBasePositionAndOrientation(ghost, pos, quat)
