"""Keyboard pick-and-place with the force gripper on a Franka Panda.

Uses the panda-gym PickAndPlace scene (table, block, and a green target
marker): drive the end-effector with the keyboard, grab the block, and bring
it to the target. Prints SUCCESS when the block reaches the target.

The arm is driven by IK on a target pose: position plus three euler angles
applied in the tool frame on top of the fingers-down base orientation.

Controls
--------
  Up / Down arrow    : move end-effector forward / backward (x)
  Left / Right arrow : move end-effector left / right (y)
  A / Z              : move end-effector up / down
  Y / U              : rotate about tool x
  H / J              : rotate about tool y (finger axis)
  N / M              : rotate about tool z (rail axis)
  K                  : reset rotation (fingers straight down)
  O / C              : open / close the gripper
  V                  : cycle display mode (visual + collision / visual / collision)
  R                  : reset (new episode: new block and target positions)
  Q / Esc            : quit

(W is avoided on purpose: PyBullet's GUI reserves it for wireframe display.)
"""

import time

import numpy as np
import pybullet as p

from collision_visual import CollisionVisualizer
from panda_force_gripper import SPAN, URDF_PATH, PandaForceGripperPickAndPlaceEnv

env = PandaForceGripperPickAndPlaceEnv(render_mode="human", control_type="ee")
robot = env.robot
client = env.sim.physics_client

robot_id = env.sim._bodies_idx["panda_force_gripper"]
collision_vis = CollisionVisualizer(robot_id, URDF_PATH)

# Fingers-down base orientation of the grasp_target frame: the fingers extend
# along the frame's +y axis, so Rx(-90 deg) points them at the floor (and lays
# the slide rail along world y).
Q_DOWN = p.getQuaternionFromEuler([-np.pi / 2, 0.0, 0.0])

# Start pose: centered over the table, well above the block.
START_POS = np.array([0.0, 0.0, 0.30])
WORKSPACE_LO = np.array([-0.35, -0.35, 0.005])
WORKSPACE_HI = np.array([0.35, 0.35, 0.50])

MOVE_STEP = 0.005    # m per loop tick while a key is held
EULER_STEP = 0.02    # rad per loop tick
GRIP_STEP = 0.004    # m of width per loop tick


def held(keys, key):
    return key in keys and keys[key] & p.KEY_IS_DOWN


def tapped(keys, key):
    return key in keys and keys[key] & p.KEY_WAS_TRIGGERED


def drive(pos, euler, width):
    """IK the arm to the target pose and set the finger width."""
    q_local = p.getQuaternionFromEuler(list(euler))
    orientation = p.multiplyTransforms([0, 0, 0], Q_DOWN, [0, 0, 0], q_local)[1]
    angles = robot.inverse_kinematics(
        link=robot.ee_link, position=pos, orientation=np.array(orientation)
    )
    finger_q = (SPAN - width) / 2
    robot.control_joints(target_angles=np.concatenate((angles[:7], [finger_q, finger_q])))


def new_episode():
    env.reset()
    pos, euler, width = START_POS.copy(), np.zeros(3), SPAN
    # Settle into the start pose before handing over control, so the episode
    # begins with the gripper hovering fingers-down above the table.
    for _ in range(90):
        drive(pos, euler, width)
        env.sim.step()
    collision_vis.sync()
    return pos, euler, width


pos, euler, width = new_episode()
print(__doc__)

running = True
announced = False
while running:
    keys = client.getKeyboardEvents()

    if held(keys, p.B3G_UP_ARROW):
        pos[0] += MOVE_STEP
    if held(keys, p.B3G_DOWN_ARROW):
        pos[0] -= MOVE_STEP
    if held(keys, p.B3G_LEFT_ARROW):
        pos[1] += MOVE_STEP
    if held(keys, p.B3G_RIGHT_ARROW):
        pos[1] -= MOVE_STEP
    if held(keys, ord("a")):
        pos[2] += MOVE_STEP
    if held(keys, ord("z")):
        pos[2] -= MOVE_STEP

    for axis, (key_plus, key_minus) in enumerate([("y", "u"), ("h", "j"), ("n", "m")]):
        if held(keys, ord(key_plus)):
            euler[axis] += EULER_STEP
        if held(keys, ord(key_minus)):
            euler[axis] -= EULER_STEP
    if tapped(keys, ord("k")):
        euler[:] = 0.0

    if held(keys, ord("o")):
        width += GRIP_STEP
    if held(keys, ord("c")):
        width -= GRIP_STEP

    if tapped(keys, ord("r")):
        pos, euler, width = new_episode()
        announced = False
    ESCAPE = 27
    if tapped(keys, ord("q")) or tapped(keys, ESCAPE):
        running = False

    pos = np.clip(pos, WORKSPACE_LO, WORKSPACE_HI)
    width = float(np.clip(width, 0.0, SPAN))

    drive(pos, euler, width)
    env.sim.step()
    collision_vis.handle_keys(keys)
    collision_vis.sync()

    success = bool(env.task.is_success(env.task.get_achieved_goal(), env.task.get_goal()))
    if success and not announced:
        print("SUCCESS! Block delivered to the target. Press R for a new episode.")
        announced = True

    time.sleep(1.0 / 30.0)

env.close()
