#!/usr/bin/env python3
"""Keep sending heartbeats to PX4 on 18570 (instance 0 / drone1) and print anything that arrives on 14550."""
import socket
import time
import threading

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 14550))
s.settimeout(0.5)

hb = bytes([0xFE, 0x09, 0x00, 0xFF, 0xBE, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03,
            0xF6, 0xA8])

def sender():
    while True:
        s.sendto(hb, ("127.0.0.1", 18570))
        print("→ sent heartbeat to 18570")
        time.sleep(1)

t = threading.Thread(target=sender, daemon=True)
t.start()

print("Bound to 14550, sending heartbeats to PX4:18570 — waiting for data...")
received = 0
for _ in range(60):   # 30 seconds
    try:
        data, addr = s.recvfrom(4096)
        print(f"← Got {len(data)} bytes from {addr}: {data[:8].hex()}")
        received += 1
        if received >= 5:
            break
    except socket.timeout:
        pass

if received == 0:
    print("No data received in 30s — PX4 is not sending to 14550")
