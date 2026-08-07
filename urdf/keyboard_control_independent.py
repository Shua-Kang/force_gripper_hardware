"""
Independent keyboard control for the force_control gripper in PyBullet.

Unlike keyboard_control.py (both fingers driven together), this script lets the
LEFT and RIGHT fingertips each travel the *whole* gripper span independently.

Span model
----------
Think of the gripper opening as a line from 0 (left end) to 1 (right end).
  - left  fingertip starts at s = 0
  - right fingertip starts at s = 1
Each fingertip can be moved anywhere in [0, 1] -- so one finger can travel all
the way across to the other side. They do NOT pass through each other: self
collision stops them when the two fingertips touch.

  joint travel for each finger is the full span (URDF limit 0 .. 0.0992):
    left  joint target = s_left  * SPAN          (s=0 -> 0,      s=1 -> 0.0992)
    right joint target = (1 - s_right) * SPAN     (s=1 -> 0,      s=0 -> 0.0992)
  "closed" (fingers meeting in the middle) = both at s = 0.5.

Controls
--------
  LEFT fingertip
    A          : move left fingertip toward the right (hold)
    Z          : move left fingertip toward the left  (hold)

  RIGHT fingertip
    K          : move right fingertip toward the left  (hold)
    M          : move right fingertip toward the right (hold)

  BOTH
    C / Down   : close (both move toward the middle, 0.5) (hold)
    O / Up     : open  (left -> 0, right -> 1)            (hold)
    Space      : toggle fully open / closed-at-middle
    R          : reset to fully open (left=0, right=1)
    V          : cycle display mode (visual + collision / visual / collision)
    Q / Esc    : quit
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
    # URDF_USE_SELF_COLLISION so the two fingertips collide with each other
    # (they are siblings under base-1) instead of passing through. Parent-child
    # pairs (each finger vs base-1) stay un-collided by default, which is what
    # we want -- only finger-vs-finger contact matters here.
    flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION,
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

LEFT_JOINT = joint_index["base-1_Slider-15"]    # left finger
RIGHT_JOINT = joint_index["base-1_Slider-16"]   # right finger

# Full joint travel = full gripper span (matches the URDF upper limit).
SPAN = 0.0992

MAX_FORCE = 50.0
SPEED = 1.0 / 240.0  # how fast a fingertip moves along the span per held step


def clamp01(x):
    return max(0.0, min(1.0, x))


def apply_state(s_left, s_right):
    """s_left, s_right in [0, 1]: position of each fingertip along the span."""
    left_target = clamp01(s_left) * SPAN
    right_target = (1.0 - clamp01(s_right)) * SPAN
    p.setJointMotorControl2(robotID, LEFT_JOINT, p.POSITION_CONTROL,
                            targetPosition=left_target, force=MAX_FORCE)
    p.setJointMotorControl2(robotID, RIGHT_JOINT, p.POSITION_CONTROL,
                            targetPosition=right_target, force=MAX_FORCE)


def held(keys, key):
    return key in keys and keys[key] & p.KEY_IS_DOWN


def tapped(keys, key):
    return key in keys and keys[key] & p.KEY_WAS_TRIGGERED


# ---------------------------------------------------------------- loop
s_left = 0.0     # left  fingertip starts at the left end
s_right = 1.0    # right fingertip starts at the right end (fully open)
toggle_closed = True   # next space-toggle closes to the middle
apply_state(s_left, s_right)

print("\n=== Independent keyboard controls ===")
print("  LEFT : A=move right  Z=move left")
print("  RIGHT: K=move left   M=move right")
print("  BOTH : C/Down=close  O/Up=open  Space=toggle  R=reset  Q/Esc=quit")
print("  VIEW : V=cycle visual/collision display")
print("  (fingertips collide -- they won't pass through each other)\n")

running = True
while running:
    keys = p.getKeyboardEvents()

    # ----- LEFT fingertip (held) -----
    if held(keys, ord('a')):
        s_left += SPEED
    if held(keys, ord('z')):
        s_left -= SPEED

    # ----- RIGHT fingertip (held) -----
    if held(keys, ord('k')):
        s_right -= SPEED
    if held(keys, ord('m')):
        s_right += SPEED

    # ----- BOTH (held): close toward middle / open to the ends -----
    if held(keys, ord('c')) or held(keys, p.B3G_DOWN_ARROW):
        s_left += SPEED
        s_right -= SPEED
    if held(keys, ord('o')) or held(keys, p.B3G_UP_ARROW):
        s_left -= SPEED
        s_right += SPEED

    # ----- BOTH (tap) -----
    if tapped(keys, ord(' ')):
        if toggle_closed:
            s_left = s_right = 0.5
        else:
            s_left, s_right = 0.0, 1.0
        toggle_closed = not toggle_closed
    if tapped(keys, ord('r')):
        s_left, s_right = 0.0, 1.0

    ESCAPE = 27
    if tapped(keys, ord('q')) or tapped(keys, ESCAPE):
        running = False

    s_left = clamp01(s_left)
    s_right = clamp01(s_right)
    apply_state(s_left, s_right)
    collision_vis.handle_keys(keys)

    p.stepSimulation()
    collision_vis.sync()
    time.sleep(1.0 / 240.0)

p.disconnect()
