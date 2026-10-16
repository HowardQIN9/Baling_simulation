#!/usr/bin/env python3
"""Launch the normal world with bale_1 already aligned between the open jaws."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    build = root / "build"
    if not (build / "libBaleLoaderHydraulics.so").is_file():
        raise SystemExit("Build first: cmake -S . -B build && cmake --build build -j 2")
    tree = ET.parse(root / "worlds/simple_tractor.sdf")
    world = tree.getroot().find("world")
    carrier = world.find("model[@name='simple_tractor']/link[@name='attachment_carrier']")
    fx = float(carrier.findtext("pose").split()[0])
    world.find("model[@name='bale_1']/pose").text = f"{fx+1.5} 0 0.45 0 0 0"
    env = dict(os.environ)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(build) + (
        os.pathsep + env["GZ_SIM_SYSTEM_PLUGIN_PATH"]
        if env.get("GZ_SIM_SYSTEM_PLUGIN_PATH") else "")
    print("Aligned grip demo: bale starts on the ground between open jaws.", flush=True)
    print("Use P to close, N to hold, U to raise; lower before opening with O.", flush=True)
    with tempfile.TemporaryDirectory(prefix="bale-grip-demo-") as tmp:
        path = Path(tmp) / "world.sdf"
        tree.write(path, encoding="unicode")
        process = subprocess.Popen(["gz", "sim", "-r", *sys.argv[1:], str(path)], env=env)
        try:
            return process.wait()
        except KeyboardInterrupt:
            # The child also receives terminal Ctrl-C. Let it close cleanly first.
            try:
                return process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    return process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    return process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
