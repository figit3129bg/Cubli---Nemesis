#!/usr/bin/env python3
"""
Reads the CSV lines the Teensy prints over USB serial (see cubli_teensy.cpp)
and writes them to cubli_log.csv on the PC, in the cubo_teensy folder.

This exists because the Teensy itself has no filesystem to write a .csv
file to -- it can only stream data out over serial. This script is the
PC-side counterpart that turns that stream into the same kind of log file
that host/lqr/main.cpp writes directly with std::ofstream.

Requires: pip install pyserial

Usage:
    python log_serial.py [PORT] [BAUD]

PORT defaults to /dev/ttyACM0 (typical Teensy USB serial device on
Linux/Mac). On Windows, pass something like COM5. BAUD defaults to 115200
but is actually irrelevant for Teensy's native USB serial (it runs at full
USB speed regardless of the requested baud) -- kept as an argument for
consistency and in case the wiring changes to a real UART later.
"""
import sys
import csv
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
OUTPUT_PATH = "cubli_log.csv"
HEADER = ["time_s", "theta_b", "theta_b_dot", "theta_w_dot", "torque"]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    ser = serial.Serial(port, baud, timeout=1)
    print(f"Connected to {port} @ {baud} baud. Logging to {OUTPUT_PATH} ... Ctrl+C to stop.")

    with open(OUTPUT_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        f.flush()

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                if raw.startswith("#"):
                    # Status line from the Teensy (e.g. "no response from
                    # moteus!" or a CAN error) -- not a data row.
                    print(raw)
                    continue

                if raw.replace(" ", "") == ",".join(HEADER):
                    # The header row the Teensy prints once in setup().
                    continue

                fields = raw.split(",")
                if len(fields) != len(HEADER):
                    continue  # malformed/partial line, e.g. right after connecting

                try:
                    [float(x) for x in fields]  # sanity check they're numeric
                except ValueError:
                    continue

                writer.writerow(fields)
                f.flush()
                print(raw)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()