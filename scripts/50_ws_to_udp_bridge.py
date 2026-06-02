"""Receive Quest Browser WebSocket packets and forward them to local UDP."""

import argparse
import asyncio
import socket

import websockets


parser = argparse.ArgumentParser()
parser.add_argument("--ws-ip", default="0.0.0.0")
parser.add_argument("--ws-port", type=int, default=8765)
parser.add_argument("--udp-ip", default="127.0.0.1")
parser.add_argument("--udp-port", type=int, default=5005)
args = parser.parse_args()

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


async def handler(websocket):
    print("Quest Browser connected")

    async for message in websocket:
        print("WS:", message)
        udp_sock.sendto(message.encode("utf-8"), (args.udp_ip, args.udp_port))


async def main():
    print(f"WebSocket listening on {args.ws_ip}:{args.ws_port}")
    print(f"Forwarding to UDP {args.udp_ip}:{args.udp_port}")

    async with websockets.serve(handler, args.ws_ip, args.ws_port):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())