"""
Keyboard control for the force_control gripper in PyBullet.

Controls
--------
  O / Up arrow     : open the gripper
  C / Down arrow   : close the gripper
  Space            : toggle fully open / fully closed
  R                : reset to closed
  V                : cycle display mode (visual + collision / visual / collision)
  Q / Esc          : quit

The two fingers are driven symmetrically:
  - left  finger joint (base-1_Slider-13): 0.0 (closed) -> -0.1 (open)
  - right finger joint (base-1_Slider-14): 0.0 (closed) -> +0.1 (open)
A single "opening" value in [0, 1] maps to both joints.
"""

import os
import time
import pybullet as p
import pybullet_data

from collision_visual import CollisionVisualizer

# ---------------------------------------------------------------- setup
HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "force_control_gripper.urdf")

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.2]
startOrn = p.getQuaternionFromEuler([0, 0, 0])
# useFixedBase=True so the gripper body stays put while we drive the fingers.
robotID = p.loadURDF(
    URDF_PATH,
    startPos,
    startOrn,
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE,
)

# Green ghost overlay of the URDF <collision> meshes; V cycles display modes.
collision_vis = CollisionVisualizer(robotID, URDF_PATH)

# ---------------------------------------------------------------- joints
# Map joint names -> index so we don't hard-code indices.
joint_index = {}
for i in range(p.getNumJoints(robotID)):
    info = p.getJointInfo(robotID, i)
    name = info[1].decode("utf-8")
    joint_index[name] = i
    print(f"joint {i}: {name}  type={info[2]}  limits=({info[8]}, {info[9]})")

LEFT_JOINT = joint_index["base-1_Slider-15"]    # left finger,  range 0.0 -> 0.0496
RIGHT_JOINT = joint_index["base-1_Slider-16"]   # right finger, range 0.0 -> 0.0496

# Both joints share axis Z; the right joint frame is flipped 180 deg about X,
# so driving both to the same value moves the fingers symmetrically.
# If open/closed end up swapped, just exchange these two values.
LEFT_OPEN, LEFT_CLOSED = 0.0, 0.0496
RIGHT_OPEN, RIGHT_CLOSED = 0.0, 0.0496

MAX_FORCE = 50.0
SPEED = 1.0 / 240.0  # how fast "opening" changes per simulation step when held


def apply_opening(opening):
    """opening in [0, 1]: 0 = closed, 1 = fully open."""
    opening = max(0.0, min(1.0, opening))
    left_target = LEFT_CLOSED + opening * (LEFT_OPEN - LEFT_CLOSED)
    right_target = RIGHT_CLOSED + opening * (RIGHT_OPEN - RIGHT_CLOSED)
    p.setJointMotorControl2(robotID, LEFT_JOINT, p.POSITION_CONTROL,
                            targetPosition=left_target, force=MAX_FORCE)
    p.setJointMotorControl2(robotID, RIGHT_JOINT, p.POSITION_CONTROL,
                            targetPosition=right_target, force=MAX_FORCE)


# ---------------------------------------------------------------- loop
opening = 0.0          # start closed
toggle_target = 1.0    # next space-toggle goes to open
apply_opening(opening)

print("\n=== Keyboard controls ===")
print("  O / Up    : open")
print("  C / Down  : close")
print("  Space     : toggle open/closed")
print("  R         : reset (closed)")
print("  V         : cycle visual/collision display")
print("  Q / Esc   : quit\n")

running = True
while running:
    keys = p.getKeyboardEvents()

    # Held keys: O / up-arrow to open, C / down-arrow to close.
    if (ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN) or \
       (p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN):
        opening += SPEED
    if (ord('c') in keys and keys[ord('c')] & p.KEY_IS_DOWN) or \
       (p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN):
        opening -= SPEED

    # Tap keys (trigger once on press).
    if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
        opening = toggle_target
        toggle_target = 0.0 if toggle_target == 1.0 else 1.0
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
        opening = 0.0
    ESCAPE = 27
    if (ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED) or \
       (ESCAPE in keys and keys[ESCAPE] & p.KEY_WAS_TRIGGERED):
        running = False

    opening = max(0.0, min(1.0, opening))
    apply_opening(opening)
    collision_vis.handle_keys(keys)

    p.stepSimulation()
    collision_vis.sync()
    time.sleep(1.0 / 240.0)

p.disconnect()
