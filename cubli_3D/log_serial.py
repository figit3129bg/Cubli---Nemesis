#!/usr/bin/env python3
"""
Reads the CSV lines a Teensy sketch (one.cpp, main.cpp, ...) prints over USB
serial and writes them to cubli_log.csv on the PC, in this same folder.

This exists because the Teensy itself has no filesystem to write a .csv
file to -- it can only stream data out over serial. This script is the
PC-side counterpart that turns that stream into the same kind of log file
that host/lqr/main.cpp writes directly with std::ofstream.

The CSV header isn't hardcoded here -- it's recognized on the fly: any line
with commas whose fields *aren't* all numbers (e.g.
"theta_b,theta_b_dot,theta_w_dot,torque,torque_raw,delta_t") is treated as
a header, so this script works unchanged no matter which sketch/env is
flashed. Everything else non-numeric (plain status text from setup(), no
commas) is just printed, never mistaken for a header or a data row.

Requires: pip install pyserial

Usage:
    python log_serial.py [PORT] [BAUD]

PORT defaults to /dev/ttyACM0 (typical Teensy USB serial device on
Linux/Mac). On Windows, pass something like COM5. BAUD defaults to 115200
but is actually irrelevant for Teensy's native USB serial (it runs at full
USB speed regardless of the requested baud) -- kept as an argument for
consistency and in case the wiring changes to a real UART later.

Each run truncates and rewrites cubli_log.csv from scratch (opened in "w"
mode, not "a") -- so re-running this always starts a fresh log rather than
appending to the last one.
"""
import sys
import csv
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
OUTPUT_PATH = "cubli_log1.csv"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    ser = serial.Serial(port, baud, timeout=1)
    print(f"Connected to {port} @ {baud} baud. Logging to {OUTPUT_PATH} ... Ctrl+C to stop.")

    header = None  # set once a real header line (see is_header below) shows up

    def is_numeric_row(fields):
        try:
            [float(x) for x in fields]
            return True
        except ValueError:
            return False

    with open(OUTPUT_PATH, "w", newline="") as f:  # "w" -- fresh file every run
        writer = csv.writer(f)

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                fields = raw.split(",")

                if len(fields) < 2:
                    # No comma at all -- a plain status/info line from setup()
                    # (e.g. "BMI270 connected!", a CAN error, "#..." messages),
                    # not a CSV row. Just echo it.
                    print(raw)
                    continue

                if not is_numeric_row(fields):
                    # Comma-separated but not all-numeric -- this is the header
                    # (column names). The sketch only prints it once in setup(),
                    # so latch onto it whenever it happens to show up.
                    if fields != header:
                        header = fields
                        writer.writerow(header)
                        f.flush()
                        print(raw)
                    continue

                if header is None:
                    # Numeric data arrived before we ever saw a header line --
                    # e.g. this script attached after setup() already printed
                    # it once. Drop rows until the sketch reprints its header.
                    continue

                if len(fields) != len(header):
                    continue  # malformed/partial line, e.g. right after connecting

                writer.writerow(fields)
                f.flush()
                print(raw)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()