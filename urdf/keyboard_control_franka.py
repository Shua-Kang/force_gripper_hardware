"""Keyboard demo: force gripper mounted on a Franka Panda arm in PyBullet.

Run scripts/make_panda_with_gripper.py first if panda_with_force_gripper.urdf
does not exist yet.

Controls
--------
  O / Up arrow     : open the gripper
  C / Down arrow   : close the gripper
  Space            : toggle fully open / fully closed
  J / L            : rotate arm joint 1 (base yaw)
  I / K            : rotate arm joint 4 (elbow)
  R                : reset arm to neutral pose, gripper open
  V                : cycle display mode (visual + collision / visual / collision)
  Q / Esc          : quit
"""

import os
import time

import pybullet as p
import pybullet_data

from collision_visual import CollisionVisualizer

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "panda_with_force_gripper.urdf")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")

robotID = p.loadURDF(URDF_PATH, [0, 0, 0], useFixedBase=True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION)

collision_vis = CollisionVisualizer(robotID, URDF_PATH)

ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
ARM_NEUTRAL = [0.00, 0.41, 0.00, -1.85, 0.00, 2.26, 0.79]
LEFT_JOINT, RIGHT_JOINT = 9, 10   # base-1_Slider-15 / -16
SPAN = 0.0992                     # fingers meet when q_left + q_right = SPAN

ARM_FORCES = [87.0, 87.0, 87.0, 87.0, 12.0, 120.0, 120.0]
FINGER_FORCE = 50.0
SPEED = 1.0 / 240.0


def apply(arm, opening):
    """arm: 7 joint angles; opening in [0, 1], 1 = fully open."""
    for j, q, f in zip(ARM_JOINTS, arm, ARM_FORCES):
        p.setJointMotorControl2(robotID, j, p.POSITION_CONTROL, targetPosition=q, force=f)
    finger_q = (1.0 - opening) * SPAN / 2
    for j in (LEFT_JOINT, RIGHT_JOINT):
        p.setJointMotorControl2(robotID, j, p.POSITION_CONTROL,
                                targetPosition=finger_q, force=FINGER_FORCE)


def held(keys, key):
    return key in keys and keys[key] & p.KEY_IS_DOWN


def tapped(keys, key):
    return key in keys and keys[key] & p.KEY_WAS_TRIGGERED


arm = list(ARM_NEUTRAL)
for j, q in zip(ARM_JOINTS, arm):
    p.resetJointState(robotID, j, q)
opening = 1.0
toggle_target = 0.0
apply(arm, opening)

print(__doc__)

running = True
while running:
    keys = p.getKeyboardEvents()

    if held(keys, ord("o")) or held(keys, p.B3G_UP_ARROW):
        opening += SPEED * 4
    if held(keys, ord("c")) or held(keys, p.B3G_DOWN_ARROW):
        opening -= SPEED * 4
    if held(keys, ord("j")):
        arm[0] += SPEED * 2
    if held(keys, ord("l")):
        arm[0] -= SPEED * 2
    if held(keys, ord("i")):
        arm[3] += SPEED * 2
    if held(keys, ord("k")):
        arm[3] -= SPEED * 2

    if tapped(keys, ord(" ")):
        opening = toggle_target
        toggle_target = 1.0 - toggle_target
    if tapped(keys, ord("r")):
        arm = list(ARM_NEUTRAL)
        opening = 1.0

    ESCAPE = 27
    if tapped(keys, ord("q")) or tapped(keys, ESCAPE):
        running = False

    opening = max(0.0, min(1.0, opening))
    apply(arm, opening)
    collision_vis.handle_keys(keys)

    p.stepSimulation()
    collision_vis.sync()
    time.sleep(1.0 / 240.0)

p.disconnect()
