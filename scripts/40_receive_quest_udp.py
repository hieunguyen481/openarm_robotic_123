"""Receive and print Quest controller packets over UDP."""

import argparse

from stage5_quest_common import (
    DEFAULT_UDP_IP,
    DEFAULT_UDP_PORT,
    make_udp_socket,
    parse_quest_packet,
)

parser = argparse.ArgumentParser()
parser.add_argument("--ip", default=DEFAULT_UDP_IP)
parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT)
parser.add_argument("--once", action="store_true")
parser.add_argument("--timeout", type=float, default=None)
args = parser.parse_args()

sock = make_udp_socket(args.ip, args.port)
if args.timeout is not None:
    sock.settimeout(args.timeout)
print(f"Listening Quest UDP on {args.ip}:{args.port} ...")
print("Expected JSON: {'x': 0.0, 'y': 0.0, 'z': 0.0, 'grip': 0.0}")

while True:
    try:
        data, addr = sock.recvfrom(4096)
    except TimeoutError:
        print("timeout: no UDP packet received")
        break

    packet = parse_quest_packet(data)
    if packet is None:
        print("raw from", addr, data.decode("utf-8", errors="replace"))
    else:
        print("from", addr, packet)

    if args.once:
        break
