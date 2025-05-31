#!/usr/bin/env python3
import argparse
import sys
import time
import threading

import serial
import matplotlib.pyplot as plt

def parse_args():
    p = argparse.ArgumentParser(
        description="Read angle data from /dev/ttyACM0 (115200 baud) and plot @3:<angle> over time."
    )
    p.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Serial port to open (default: /dev/ttyACM0)",
    )
    p.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Baud rate for the serial port (default: 115200)",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="How many seconds to collect before plotting (default: 10.0)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds (default: 0.1)",
    )
    return p.parse_args()

def serial_reader(port, baudrate, timeout, stop_event, data_list):
    """
    Open serial port, read lines, parse @3:<angle>;;, append (timestamp, angle) to data_list.
    Stops when stop_event.is_set().
    """
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error: could not open serial port {port}: {e}")
        sys.exit(1)

    # Make sure the input buffer is clear
    ser.reset_input_buffer()

    while not stop_event.is_set():
        try:
            line_bytes = ser.readline()
        except serial.SerialException:
            # In case port gets disconnected or other error
            break

        if not line_bytes:
            # Timeout hit – loop again to check stop_event
            continue

        try:
            line = line_bytes.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        # We're specifically looking for lines that start with "@3:"
        # Format: @3:<angle>;;
        if line.startswith("@3:"):
            # Remove the "@3:" prefix and any trailing delimiters
            # e.g. "@3:123.456;;" → "123.456;;" → float("123.456")
            payload = line[3:]
            # Strip trailing semicolons if present
            payload = payload.rstrip(";")
            try:
                angle = float(payload)
                print(f"Read angle: {angle}°")
                timestamp = time.time()
                data_list.append((timestamp, angle))
            except ValueError:
                # Failed to parse float (malformed), ignore
                continue

    ser.close()

def main():
    args = parse_args()

    # Shared list for (timestamp, angle)
    data = []

    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baudrate, args.timeout, stop_event, data),
        daemon=True,
    )
    print(f"Opening serial port {args.port} @ {args.baudrate} baud, reading for {args.duration} s...")
    reader_thread.start()

    # Sleep for the given duration, then signal the thread to stop
    time.sleep(args.duration)
    stop_event.set()
    reader_thread.join()

    if len(data) == 0:
        print("No @3: data found in the specified duration.")
        sys.exit(0)

    # Separate into two lists: elapsed time (secs) and angle (deg)
    t0 = data[0][0]
    times = [(t - t0) for (t, _) in data]
    angles = [angle for (_, angle) in data]

    # Plotting
    plt.figure(figsize=(8, 4))
    plt.plot(times, angles, marker="o", linestyle="-", markersize=4)
    plt.title(f"Angle (@3:) vs Time (duration={args.duration}s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
