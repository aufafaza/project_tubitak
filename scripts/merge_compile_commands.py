#!/usr/bin/env python3
"""Merge per-package compile_commands.json into workspace root after colcon build."""
import json
import glob
import os
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
files = glob.glob(os.path.join(root, "build", "*", "compile_commands.json"))

if not files:
    print("No compile_commands.json files found. Run:")
    print("  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
    sys.exit(1)

merged = []
for f in files:
    with open(f) as fh:
        merged.extend(json.load(fh))

out = os.path.join(root, "compile_commands.json")
with open(out, "w") as fh:
    json.dump(merged, fh, indent=2)

print(f"Merged {len(merged)} entries from {len(files)} packages → {out}")
