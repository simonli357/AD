#!/usr/bin/env python3
"""Probe the BFMC TrafficCommunicationServer without launching ROS."""

import argparse
import json
import socket
import sys
import time


def parse_json_stream(buffer):
    decoder = json.JSONDecoder()
    idx = 0
    messages = []

    while idx < len(buffer):
        next_start = buffer.find("{", idx)
        if next_start == -1:
            return messages, ""

        try:
            message, end = decoder.raw_decode(buffer[next_start:])
        except json.JSONDecodeError:
            return messages, buffer[next_start:]

        messages.append(message)
        idx = next_start + end

    return messages, ""


def discover_server(timeout, udp_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind(("", udp_port))

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            datagram, address = sock.recvfrom(4096)
        except socket.timeout:
            break

        parts = datagram.split(b"(-.-)")
        payload = parts[-1]
        try:
            text = payload.decode()
        except UnicodeDecodeError:
            continue

        if ":" not in text:
            continue

        _, port_text = text.rsplit(":", 1)
        try:
            return address[0], int(port_text), text
        except ValueError:
            continue

    return None, None, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.50.2", help="Traffic server IP")
    parser.add_argument("--port", type=int, default=5000, help="Traffic server TCP port")
    parser.add_argument("--loc-id", type=int, default=10, help="Localization tag/device ID")
    parser.add_argument("--freq", type=float, default=0.25, help="Location subscription period in seconds")
    parser.add_argument("--timeout", type=float, default=5.0, help="TCP connect/read timeout")
    parser.add_argument("--discover", action="store_true", help="Listen for the signed UDP server broadcast first")
    parser.add_argument("--udp-port", type=int, default=9000, help="UDP discovery port")
    args = parser.parse_args()

    host, port = args.host, args.port
    if args.discover:
        print(f"Listening for UDP discovery on port {args.udp_port} for {args.timeout:.1f}s...")
        found_host, found_port, payload = discover_server(args.timeout, args.udp_port)
        if found_host is None:
            print("No TrafficCommunicationServer UDP broadcast received.")
        else:
            host, port = found_host, found_port
            print(f"Discovered server {host}:{port} from payload {payload!r}")

    subscription = {
        "reqORinfo": "info",
        "type": "locIDsub",
        "locID": args.loc_id,
        "freq": args.freq,
    }

    print(f"Connecting to {host}:{port}...")
    try:
        with socket.create_connection((host, port), timeout=args.timeout) as sock:
            sock.settimeout(args.timeout)
            payload = json.dumps(subscription, separators=(",", ":")).encode()
            print(f"Sending subscription: {payload.decode()}")
            sock.sendall(payload)

            buffer = ""
            deadline = time.time() + args.timeout
            saw_location = False
            saw_error = False

            while time.time() < deadline:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    break

                if not chunk:
                    print("Server closed the connection.")
                    break

                text = chunk.decode(errors="replace")
                print(f"RAW {text!r}")
                buffer += text
                messages, buffer = parse_json_stream(buffer)

                for message in messages:
                    print("JSON " + json.dumps(message, sort_keys=True))
                    if message.get("error"):
                        saw_error = True
                    if message.get("type") == "location" and "x" in message and "y" in message:
                        saw_location = True

            if saw_location:
                print("RESULT: connected and received location data")
                return 0
            if saw_error:
                print("RESULT: connected, but server returned an error")
                return 2

            print("RESULT: connected, but no location data arrived before timeout")
            return 3
    except OSError as exc:
        print(f"RESULT: connection failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
