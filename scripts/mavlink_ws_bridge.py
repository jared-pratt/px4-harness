#!/usr/bin/env python3
"""
MAVLink UDP ↔ WebSocket bridge for ADOS Mission Control.

How PX4 SITL multi-instance MAVLink works
------------------------------------------
Each PX4 instance listens for commands on its own port (18570, 18571, …)
and sends MAVLink telemetry back to whoever last sent it a packet.
By binding our local UDP socket to the standard GCS receive ports
(14550, 14551) and sending a heartbeat FROM those ports to PX4, PX4
records our address and streams telemetry back to us.

Port mapping
------------
  Drone 1:  local 14550 ↔ PX4 18570  |  ws://0.0.0.0:5760
  Drone 2:  local 14551 ↔ PX4 18571  |  ws://0.0.0.0:5761

Usage
-----
  python scripts/mavlink_ws_bridge.py [--host HOST]

In ADOS → Connect → WebSocket → ws://localhost:5760  (drone 1)
                                  ws://localhost:5761  (drone 2)

Requirements
------------
  pip install websockets pymavlink
"""
from __future__ import annotations

import argparse
import asyncio
import sys
import time
from typing import Set, Tuple

try:
    import websockets
except ImportError:
    sys.exit("websockets not installed.  Run: pip install websockets pymavlink")

try:
    from pymavlink import mavutil
    _HAS_PYMAVLINK = True
except ImportError:
    _HAS_PYMAVLINK = False


BRIDGES = [
    {"label": "drone1", "local_port": 14550, "px4_port": 18570, "ws_port": 5760},
    {"label": "drone2", "local_port": 14551, "px4_port": 18571, "ws_port": 5761},
    {"label": "drone3", "local_port": 14552, "px4_port": 18572, "ws_port": 5762},
]


def _ts() -> str:
    return time.strftime("%H:%M:%S")


def _make_heartbeat() -> bytes:
    """Return a serialised MAVLink v1 GCS heartbeat using pymavlink."""
    if not _HAS_PYMAVLINK:
        # Fallback: pre-built valid heartbeat bytes (sysid=255, compid=190)
        return bytes([
            0xFE, 0x09, 0x00, 0xFF, 0xBE, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03,
            0xF6, 0xA8,
        ])
    mav = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
    mav.robust_parsing = True
    msg = mav.heartbeat_encode(
        type=mavutil.mavlink.MAV_TYPE_GCS,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
        mavlink_version=3,
    )
    return msg.pack(mav)


class _UDPProtocol(asyncio.DatagramProtocol):
    """
    Asyncio DatagramProtocol that receives PX4 telemetry and fans it out
    to all connected WebSocket clients.  Sending heartbeats/commands back
    to PX4 is done via transport.sendto().
    """

    def __init__(self, label: str, clients: Set, px4_addr: Tuple[str, int]) -> None:
        self.label = label
        self.clients = clients
        self.px4_addr = px4_addr
        self.transport: asyncio.DatagramTransport | None = None
        self._count = 0

    def connection_made(self, transport: asyncio.DatagramTransport) -> None:  # type: ignore[override]
        self.transport = transport

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        self._count += 1
        if self._count <= 5 or self._count % 100 == 0:
            print(f"[{_ts()}] [{self.label}] ← UDP #{self._count} {len(data)}b from {addr}")
        if self.clients:
            dead: Set = set()
            for ws in list(self.clients):
                try:
                    asyncio.ensure_future(ws.send(data))
                except Exception:
                    dead.add(ws)
            self.clients -= dead

    def send_to_px4(self, data: bytes) -> None:
        if self.transport:
            self.transport.sendto(data, self.px4_addr)

    def error_received(self, exc: Exception) -> None:
        print(f"[{_ts()}] [{self.label}] UDP error: {exc}")

    def connection_lost(self, exc: Exception | None) -> None:
        pass


async def run_bridge(
    label: str,
    local_port: int,
    px4_port: int,
    ws_port: int,
    host: str,
) -> None:
    """
    Bridge one PX4 GCS link ↔ WebSocket.

    • Binds UDP on local_port via DatagramProtocol — PX4 telemetry arrives here.
    • Sends heartbeats TO PX4 on px4_port so PX4 learns our address.
    • Serves a WebSocket on ws_port; forwards bytes in both directions.
    """
    px4_addr: Tuple[str, int] = ("127.0.0.1", px4_port)
    clients: Set = set()
    heartbeat = _make_heartbeat()

    print(f"[{_ts()}] [{label}]  local UDP :{local_port}  ←→  PX4 127.0.0.1:{px4_port}  |  WS {host}:{ws_port}")

    loop = asyncio.get_running_loop()
    _, protocol = await loop.create_datagram_endpoint(
        lambda: _UDPProtocol(label, clients, px4_addr),
        local_addr=("0.0.0.0", local_port),
    )
    udp: _UDPProtocol = protocol  # type: ignore[assignment]

    # ------------------------------------------------------------------ #
    # WebSocket handler                                                    #
    # ------------------------------------------------------------------ #
    async def ws_handler(ws) -> None:
        clients.add(ws)
        peer = ws.remote_address
        print(f"[{_ts()}] [{label}] ADOS connected  {peer}")
        try:
            async for msg in ws:
                if isinstance(msg, (bytes, bytearray)):
                    udp.send_to_px4(bytes(msg))
        except Exception:
            pass
        finally:
            clients.discard(ws)
            print(f"[{_ts()}] [{label}] ADOS disconnected  {peer}")

    # ------------------------------------------------------------------ #
    # Heartbeat loop — tells PX4 to stream back to local_port            #
    # ------------------------------------------------------------------ #
    _hb_count = 0

    async def heartbeat_loop() -> None:
        nonlocal _hb_count
        while True:
            udp.send_to_px4(heartbeat)
            _hb_count += 1
            if _hb_count <= 5 or _hb_count % 10 == 0:
                print(f"[{_ts()}] [{label}] → heartbeat #{_hb_count} → {px4_addr}")
            await asyncio.sleep(1.0)

    server = await websockets.serve(ws_handler, host, ws_port)
    print(f"[{_ts()}] [{label}] WebSocket listening on ws://{host}:{ws_port}")

    await asyncio.gather(
        server.wait_closed(),
        heartbeat_loop(),
    )


async def main(host: str) -> None:
    print(f"[{_ts()}] MAVLink WebSocket bridge starting — Ctrl+C to stop\n")
    await asyncio.gather(*[
        run_bridge(
            label=b["label"],
            local_port=b["local_port"],
            px4_port=b["px4_port"],
            ws_port=b["ws_port"],
            host=host,
        )
        for b in BRIDGES
    ])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAVLink UDP ↔ WebSocket bridge")
    parser.add_argument("--host", default="0.0.0.0",
                        help="WebSocket bind host (default: 0.0.0.0)")
    args = parser.parse_args()
    try:
        asyncio.run(main(args.host))
    except KeyboardInterrupt:
        print("\nBridge stopped.")
