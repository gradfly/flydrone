from .base import Drone, DryRunDrone
from .serial_link import SerialDrone
from .sim import OBSTACLE_COUNT, Box, Room, SimDrone, make_obstacles


def make_drone(kind: str, **kw) -> Drone:
    kind = kind.lower()
    if kind == "sim":
        return SimDrone(**kw)
    if kind == "tello":
        from .tello import TelloDrone

        return TelloDrone(**kw)
    if kind == "crazyflie":
        from .crazyflie import CrazyflieDrone

        return CrazyflieDrone(**kw)
    if kind == "mavlink":
        from .mavlink import MavlinkDrone

        return MavlinkDrone(**kw)
    if kind in ("esp32", "udp", "betaflight"):
        from .udp_bridge import UDPBridgeDrone

        return UDPBridgeDrone(**kw)
    if kind in ("serial", "uart", "usb"):
        from .serial_link import SerialDrone

        return SerialDrone(**kw)
    raise ValueError(f"unknown drone backend: {kind}")


__all__ = ["Box", "Drone", "DryRunDrone", "OBSTACLE_COUNT", "Room", "SerialDrone", "SimDrone",
           "make_drone", "make_obstacles"]
