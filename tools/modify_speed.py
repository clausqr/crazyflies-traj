#!/usr/bin/env python3
"""
Write the fast spherical Lissajous as the slow one at twice the speed.

    python3 tools/modify_speed.py        # run from the repo root

Reads trajectories/spherical_lissajous_slow.csv and writes
trajectories/spherical_lissajous_fast.csv: the header is copied unchanged and
every data row keeps its x, y, z while its last column (t) is halved, so the
same path is flown in half the time (65 s -> 32.5 s, peak speed 0.97 -> 1.9 m/s).
Paths are relative to the working directory, so run it from the repo root.
tests/test_samples.py checks that the committed fast file matches its output.
"""
import csv

src_path = "trajectories/spherical_lissajous_slow.csv"
dst_path = "trajectories/spherical_lissajous_fast.csv"

with open(src_path, "r", encoding="utf-8") as f_in, open(
    dst_path, "w", newline="", encoding="utf-8"
) as f_out:
    reader = csv.reader(f_in)
    writer = csv.writer(f_out)

    # Copy the header (first line) unchanged
    header = next(reader)
    writer.writerow(header)

    # Halve the last column (t) of every data row: twice the speed
    for row in reader:
        if row:  # skip empty lines
            row[-1] = str(float(row[-1]) / 2)
            writer.writerow(row)
