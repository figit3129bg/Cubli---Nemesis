#!/usr/bin/env python3
"""
Plots every numeric column of a cubli CSV log (as written by log_serial.py)
on one shared plot -- one line per column, against time.

Column set is read from the CSV header, not hardcoded, so this works for
any sketch's log shape (one.cpp's theta_b,theta_b_dot,theta_w_dot,torque,
torque_raw,delta_t; main.cpp's time_s,theta_b,...; etc.).

Time axis: uses the 'time_s' column if the log has one; otherwise, if it
has a 'delta_t' column (seconds per loop iteration -- what one.cpp logs),
reconstructs time as the running total of delta_t; otherwise just falls
back to sample index. Whichever column supplies the time axis is not also
re-plotted as its own (redundant, diagonal) line -- 'delta_t' is the
exception, since delta_t-vs-cumulative-time is informative (shows loop
timing/jitter over the run) even though it built the x-axis.

Requires: pip install matplotlib

Usage:
    python plot_log.py [CSV_PATH]

CSV_PATH defaults to cubli_log1.csv (the file log_serial.py currently
writes), in this same folder.
"""
import sys
import os
import csv
import itertools
import matplotlib.pyplot as plt

DEFAULT_CSV_PATH = "cubli_log1.csv"


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_PATH

    if not os.path.exists(csv_path):
        print(f"No such file: {csv_path}")
        return

    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [row for row in reader if len(row) == len(header)]

    if not rows:
        print(f"No data rows found in {csv_path}")
        return

    columns = {name: [] for name in header}
    for row in rows:
        for name, value in zip(header, row):
            try:
                columns[name].append(float(value))
            except ValueError:
                columns[name].append(float("nan"))

    if "time_s" in columns:
        time_s = columns.pop("time_s")
        x_label = "time_s (s)"
    elif "delta_t" in columns:
        time_s = list(itertools.accumulate(columns["delta_t"]))
        x_label = "time, reconstructed from cumulative delta_t (s)"
    else:
        time_s = list(range(len(rows)))
        x_label = "sample #"

    plt.figure(figsize=(10, 6))
    for name, values in columns.items():
        plt.plot(time_s, values, label=name)

    plt.xlabel(x_label)
    plt.ylabel("value")
    plt.title(f"{os.path.basename(csv_path)} -- {len(rows)} samples")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
