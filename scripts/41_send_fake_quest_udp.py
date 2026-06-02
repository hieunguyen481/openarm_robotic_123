"""Send fake Quest controller UDP packets for local testing."""

import argparse
import json
import socket
import time

import numpy as np

from stage5_quest_common import DEFAULT_UDP_PORT

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT)
parser.add_argument("--rate", type=float, default=30.0)
parser.add_argument("--seconds", type=float, default=20.0)
args = parser.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dt = 1.0 / args.rate
start = time.time()

print(f"Sending fake Quest UDP to {args.host}:{args.port}")
while time.time() - start < args.seconds:
    t = time.time() - start
    packet = {
        "x": 0.08 * np.sin(t * 0.8),
        "y": 0.06 * np.sin(t * 1.1),
        "z": 0.08 * np.cos(t * 0.8),
        "grip": 1.0 if (int(t) % 4) >= 2 else 0.0,
    }
    sock.sendto(json.dumps(packet).encode("utf-8"), (args.host, args.port))
    time.sleep(dt)

print("Done.")
