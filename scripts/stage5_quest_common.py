"""Shared helpers for Quest UDP teleoperation scripts."""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass

import numpy as np


DEFAULT_UDP_IP = "0.0.0.0"
DEFAULT_UDP_PORT = 5005


@dataclass
class QuestControllerPacket:
    """Minimal controller packet used by the MuJoCo teleop bridge."""

    x: float
    y: float
    z: float
    grip: float


def make_udp_socket(ip: str = DEFAULT_UDP_IP, port: int = DEFAULT_UDP_PORT):
    """Create a UDP socket bound to the requested address."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    return sock


def parse_quest_packet(raw: bytes) -> QuestControllerPacket | None:
    """Parse a small JSON UDP packet from Quest or a fake sender."""
    try:
        packet = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None

    if "left" in packet and isinstance(packet["left"], dict):
        packet = packet["left"]

    try:
        return QuestControllerPacket(
            x=float(packet.get("x", 0.0)),
            y=float(packet.get("y", 0.0)),
            z=float(packet.get("z", 0.0)),
            grip=float(packet.get("grip", 0.0)),
        )
    except (TypeError, ValueError):
        return None


def quest_packet_to_robot_target(
    packet: QuestControllerPacket,
    *,
    origin: np.ndarray,
    scale: float,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    """Map Quest controller coordinates into a bounded robot XYZ target."""
    target = np.array(
        [
            origin[0] + packet.x * scale,
            origin[1] + packet.z * scale,
            origin[2] + packet.y * scale,
        ],
        dtype=float,
    )
    return np.clip(target, low, high)
