"""Teleoperate OpenArm from Quest and stream MuJoCo camera frames over WSS."""

from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import ssl
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import websockets
from PIL import Image

from stage1_common import OUTPUT_DIR
from stage2_common import jacobian_ik_step, resolve_arm
from stage3_common import (
    cube_pos,
    cube_quat,
    left_ee_pos,
    load_object_scene,
    right_ee_pos,
    settle,
)
from stage4_common import LEFT_CLOSE, LEFT_OPEN, set_gripper
from stage5_quest_common import (
    QuestControllerPacket,
    parse_quest_packet,
    quest_packet_to_robot_target,
)


parser = argparse.ArgumentParser()
parser.add_argument("--ws-ip", default="0.0.0.0")
parser.add_argument("--ws-port", type=int, default=8765)
parser.add_argument("--certfile", default="certs/quest-webxr.crt")
parser.add_argument("--keyfile", default="certs/quest-webxr.key")
parser.add_argument("--camera", default="camera_ceiling")
parser.add_argument("--height", type=int, default=360)
parser.add_argument("--width", type=int, default=480)
parser.add_argument("--stream-fps", type=float, default=15.0)
parser.add_argument("--record", action="store_true")
parser.add_argument("--record-output", default="quest_teleop_episode_000.npz")
parser.add_argument("--scale", type=float, default=0.6)
parser.add_argument("--gain", type=float, default=1.2)
parser.add_argument("--damping", type=float, default=0.04)
parser.add_argument("--max-step", type=float, default=0.08)
parser.add_argument("--origin-x", type=float, default=0.36)
parser.add_argument("--origin-y", type=float, default=0.15)
parser.add_argument("--origin-z", type=float, default=1.12)
parser.add_argument("--slow", type=float, default=1.0)
args = parser.parse_args()

latest_packet = QuestControllerPacket(x=0.0, y=0.0, z=0.0, grip=0.0)
latest_packet_lock = threading.Lock()
latest_packet_count = 0
clients: set[websockets.ServerConnection] = set()
loop_ready = threading.Event()
ws_loop: asyncio.AbstractEventLoop | None = None


async def ws_handler(websocket):
    """Receive Quest controller JSON packets."""
    clients.add(websocket)
    print("Quest Browser connected", flush=True)
    try:
        async for message in websocket:
            packet = parse_quest_packet(message.encode("utf-8"))
            if packet is None:
                continue
            with latest_packet_lock:
                global latest_packet, latest_packet_count
                latest_packet = packet
                latest_packet_count += 1
                if latest_packet_count % 30 == 1:
                    print(
                        "packet=",
                        latest_packet_count,
                        "x=",
                        f"{packet.x:.3f}",
                        "y=",
                        f"{packet.y:.3f}",
                        "z=",
                        f"{packet.z:.3f}",
                        "grip=",
                        f"{packet.grip:.2f}",
                        flush=True,
                    )
    finally:
        clients.discard(websocket)
        print("Quest Browser disconnected", flush=True)


async def ws_main() -> None:
    """Run the WSS server until the process exits."""
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(args.certfile, args.keyfile)
    print(f"WSS listening on wss://{args.ws_ip}:{args.ws_port}", flush=True)
    async with websockets.serve(ws_handler, args.ws_ip, args.ws_port, ssl=ssl_context):
        await asyncio.Future()


def start_wss_server() -> None:
    """Start the WebSocket server in a background thread."""
    def runner() -> None:
        global ws_loop
        ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(ws_loop)
        loop_ready.set()
        ws_loop.run_until_complete(ws_main())

    threading.Thread(target=runner, daemon=True).start()
    loop_ready.wait(timeout=5.0)


def jpeg_base64(rgb: np.ndarray) -> str:
    """Encode an RGB frame as a base64 JPEG string."""
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="JPEG", quality=75)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


async def send_frame_to_clients(payload: str) -> None:
    """Broadcast one frame payload to all connected Quest browsers."""
    if not clients:
        return
    stale_clients = []
    for client in tuple(clients):
        try:
            await client.send(payload)
        except websockets.ConnectionClosed:
            stale_clients.append(client)
    for client in stale_clients:
        clients.discard(client)


def queue_frame(rgb: np.ndarray) -> None:
    """Queue one rendered frame for async WSS broadcast."""
    if ws_loop is None or not clients:
        return
    payload = json.dumps(
        {
            "type": "frame",
            "encoding": "jpg_base64",
            "camera": args.camera,
            "data": jpeg_base64(rgb),
        },
        separators=(",", ":"),
    )
    asyncio.run_coroutine_threadsafe(send_frame_to_clients(payload), ws_loop)


def save_episode(record: dict[str, list[np.ndarray]]) -> None:
    """Save a raw teleop episode with LeRobot-friendly field names."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    path = OUTPUT_DIR / args.record_output
    np.savez_compressed(
        path,
        **{key: np.array(value) for key, value in record.items()},
        camera_name=np.array(args.camera),
        fps=np.array(args.stream_fps),
    )
    print(f"Saved raw teleop episode to {path}")


start_wss_server()

scene = load_object_scene()
settle(scene)
left = resolve_arm(scene.model, "left")
renderer = mujoco.Renderer(scene.model, height=args.height, width=args.width)

camera_id = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
if camera_id == -1:
    raise SystemExit(f"Camera not found: {args.camera}")

origin = np.array([args.origin_x, args.origin_y, args.origin_z])
workspace_low = np.array([0.20, 0.05, 0.95])
workspace_high = np.array([0.58, 0.32, 1.35])
target_pos = origin.copy()
stream_period = 1.0 / max(args.stream_fps, 1.0)
last_stream = 0.0

record: dict[str, list[np.ndarray]] = {
    "observation.state": [],
    "observation.images.sim": [],
    "action": [],
    "qpos": [],
    "qvel": [],
    "ctrl": [],
    "left_ee_pos": [],
    "right_ee_pos": [],
    "object_pos": [],
    "object_quat": [],
    "quest_packet": [],
    "target_pos": [],
}

print("Quest teleop stream is ready.", flush=True)
print(f"Camera: {args.camera} ({args.width}x{args.height})", flush=True)
print(f"origin = {origin}", flush=True)
print(f"workspace_low = {workspace_low}", flush=True)
print(f"workspace_high = {workspace_high}", flush=True)

try:
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        last_print = 0.0
        while viewer.is_running():
            with latest_packet_lock:
                packet = latest_packet
                packet_count = latest_packet_count

            target_pos = quest_packet_to_robot_target(
                packet,
                origin=origin,
                scale=args.scale,
                low=workspace_low,
                high=workspace_high,
            )

            scene.data.ctrl[:] = scene.home_ctrl
            gripper_value = LEFT_CLOSE if packet.grip > 0.5 else LEFT_OPEN
            set_gripper(scene, "left", gripper_value)
            distance = jacobian_ik_step(
                scene.model,
                scene.data,
                left,
                target_pos,
                gain=args.gain,
                damping=args.damping,
                max_step=args.max_step,
            )
            set_gripper(scene, "left", gripper_value)

            mujoco.mj_step(scene.model, scene.data)
            viewer.sync()

            now = time.time()
            if now - last_stream >= stream_period:
                last_stream = now
                renderer.update_scene(scene.data, camera=args.camera)
                rgb = renderer.render()
                queue_frame(rgb)

                if args.record:
                    observation_state = np.concatenate(
                        [scene.data.qpos.copy(), scene.data.qvel.copy()]
                    )
                    action = np.concatenate(
                        [target_pos.copy(), np.array([packet.grip], dtype=float)]
                    )
                    record["observation.state"].append(observation_state)
                    record["observation.images.sim"].append(rgb.copy())
                    record["action"].append(action)
                    record["qpos"].append(scene.data.qpos.copy())
                    record["qvel"].append(scene.data.qvel.copy())
                    record["ctrl"].append(scene.data.ctrl.copy())
                    record["left_ee_pos"].append(left_ee_pos(scene))
                    record["right_ee_pos"].append(right_ee_pos(scene))
                    record["object_pos"].append(cube_pos(scene))
                    record["object_quat"].append(cube_quat(scene))
                    record["quest_packet"].append(
                        np.array([packet.x, packet.y, packet.z, packet.grip])
                    )
                    record["target_pos"].append(target_pos.copy())

            if now - last_print > 1.0:
                last_print = now
                print(
                    "clients=",
                    len(clients),
                    "packets=",
                    packet_count,
                    "target=",
                    np.round(target_pos, 4),
                    "distance=",
                    f"{distance:.4f}",
                    "grip=",
                    f"{packet.grip:.2f}",
                    flush=True,
                )

            time.sleep(float(scene.model.opt.timestep) * args.slow)
finally:
    if args.record and record["action"]:
        save_episode(record)
