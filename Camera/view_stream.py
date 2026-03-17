#!/usr/bin/env python3
"""
MJPEG stream viewer for ESP32-S3 + OV5640 camera.
Uses cv2.VideoCapture with HTTP MJPEG URL for reliable playback on macOS.

Usage:
    python3 view_stream.py [ESP32_IP]

Controls:
    q / ESC  – quit
    s        – save current frame as JPEG
"""

import sys
import cv2

STREAM_PORT = 5000


def discover_esp32():
    import socket
    print("[*] Scanning local network for ESP32 MJPEG server on port", STREAM_PORT)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
    finally:
        s.close()

    prefix = ".".join(my_ip.split(".")[:3])
    print(f"[*] Local IP: {my_ip}  –  scanning {prefix}.1-254 ...")

    for i in range(1, 255):
        ip = f"{prefix}.{i}"
        try:
            sock = socket.create_connection((ip, STREAM_PORT), timeout=0.15)
            sock.close()
            print(f"[+] Found MJPEG server at {ip}:{STREAM_PORT}")
            return ip
        except (OSError, socket.timeout):
            continue
    return None


def main():
    if len(sys.argv) > 1:
        ip = sys.argv[1]
    else:
        ip = discover_esp32()
        if ip is None:
            print("[!] Could not find ESP32 on the network.")
            print("    Usage: python3 view_stream.py <ESP32_IP>")
            sys.exit(1)

    url = f"http://{ip}:{STREAM_PORT}/"
    print(f"[*] Connecting to {url} ...")

    cap = cv2.VideoCapture(url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimize display lag
    if not cap.isOpened():
        print("[!] Failed to open stream")
        sys.exit(1)

    print("[*] Stream opened – press 'q' or ESC to quit, 's' to save a frame")
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[!] Lost connection, retrying...")
                cap.release()
                cap = cv2.VideoCapture(url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue

            frame_count += 1
            cv2.imshow("ESP32-S3 OV5640 Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                fname = f"capture_{frame_count:05d}.jpg"
                cv2.imwrite(fname, frame)
                print(f"[*] Saved {fname}")

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"[*] Done – {frame_count} frames received")


if __name__ == "__main__":
    main()
