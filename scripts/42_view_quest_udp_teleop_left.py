"""Control the left OpenArm end-effector from Quest-style UDP packets."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from stage2_common import jacobian_ik_step, resolve_arm
from stage3_common import load_object_scene, settle
from stage4_common import LEFT_CLOSE, LEFT_OPEN, set_gripper
from stage5_quest_common import (
    DEFAULT_UDP_IP,
    DEFAULT_UDP_PORT,
    QuestControllerPacket,
    make_udp_socket,
    parse_quest_packet,
    quest_packet_to_robot_target,
)

parser = argparse.ArgumentParser()
parser.add_argument("--ip", default=DEFAULT_UDP_IP)
parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT)
parser.add_argument("--scale", type=float, default=0.6)
parser.add_argument("--origin-x", type=float, default=0.36)
parser.add_argument("--origin-y", type=float, default=0.15)
parser.add_argument("--origin-z", type=float, default=1.12)
parser.add_argument("--slow", type=float, default=1.0)
args = parser.parse_args()

sock = make_udp_socket(args.ip, args.port)
sock.setblocking(False)

scene = load_object_scene()
settle(scene)
left = resolve_arm(scene.model, "left")

origin = np.array([args.origin_x, args.origin_y, args.origin_z])
workspace_low = np.array([0.20, 0.05, 0.95])
workspace_high = np.array([0.58, 0.32, 1.35])
latest_packet = QuestControllerPacket(x=0.0, y=0.0, z=0.0, grip=0.0)
target_pos = origin.copy()

print(f"Listening Quest UDP on {args.ip}:{args.port}")
print("Use scripts/41_send_fake_quest_udp.py to test without Quest.")
print(f"origin = {origin}")
print(f"workspace_low = {workspace_low}")
print(f"workspace_high = {workspace_high}")

with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
    last_print = 0.0
    while viewer.is_running():
        while True:
            try:
                raw, _ = sock.recvfrom(4096)
            except BlockingIOError:
                break
            packet = parse_quest_packet(raw)
            if packet is not None:
                latest_packet = packet
                target_pos = quest_packet_to_robot_target(
                    latest_packet,
                    origin=origin,
                    scale=args.scale,
                    low=workspace_low,
                    high=workspace_high,
                )

        scene.data.ctrl[:] = scene.home_ctrl
        gripper_value = LEFT_CLOSE if latest_packet.grip > 0.5 else LEFT_OPEN
        set_gripper(scene, "left", gripper_value)
        distance = jacobian_ik_step(
            scene.model,
            scene.data,
            left,
            target_pos,
            gain=0.65,
            damping=0.05,
            max_step=0.035,
        )
        set_gripper(scene, "left", gripper_value)

        mujoco.mj_step(scene.model, scene.data)
        viewer.sync()

        now = time.time()
        if now - last_print > 1.0:
            last_print = now
            print(
                "target=",
                np.round(target_pos, 4),
                "distance=",
                f"{distance:.4f}",
                "grip=",
                f"{latest_packet.grip:.2f}",
            )

        time.sleep(float(scene.model.opt.timestep) * args.slow)
