"""Portable image records, explicit labels, and a conservative storage guard."""
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

from .transport import decode_image


class Recorder:
    def __init__(self, directory, dataset_root, rate_hz=2, budget_gb=20):
        self.directory = Path(directory)
        self.root = Path(dataset_root)
        self.root.mkdir(parents=True, exist_ok=True)
        disk = shutil.disk_usage(self.root)
        self.limit = min(int(budget_gb * 10**9), int(disk.total * .4))
        self.used = sum(p.stat().st_size for p in self.root.rglob("*") if p.is_file())
        self.rate = rate_hz
        self.last = {}
        self.count = 0
        self.records = (self.directory / "frames.jsonl").open("w")
        self.labels = (self.directory / "ground_truth.jsonl").open("w")

    def save(self, pair, rgb=None, depth=None, estimate=None):
        name, t = pair["camera"], pair["stamp"]
        if t - self.last.get(name, -1e9) < 1 / self.rate - .001:
            return False
        # Leave >=5GB free, and reserve a worst-case frame before writing.
        if self.used + 4_000_000 > self.limit or shutil.disk_usage(self.root).free < 5e9:
            raise RuntimeError("Dataset storage budget reached; no existing data deleted")
        self.last[name] = t
        rgb = decode_image(pair["rgb_message"], "rgb") if rgb is None else rgb
        depth = decode_image(pair["depth_message"], "depth") if depth is None else depth
        ident = f"{name}_{int(round(t * 1e9)):016d}"
        folder = self.directory / "images"
        folder.mkdir(exist_ok=True)
        rgb_path, depth_path = folder / (ident + ".jpg"), folder / (ident + ".npz")
        if not cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                           [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise IOError("Image write failed")
        np.savez_compressed(depth_path, depth_m=depth.astype(np.float32))
        record = {key: value for key, value in pair.items()
                  if key not in ("rgb_message", "depth_message", "ground_truth")}
        record.update(id=ident, rgb=str(rgb_path.relative_to(self.directory)),
                      depth=str(depth_path.relative_to(self.directory)), estimate=estimate)
        self.records.write(json.dumps(record, allow_nan=False) + "\n")
        self.labels.write(json.dumps(dict(id=ident, stamp=t,
                                         poses=pair["ground_truth"])) + "\n")
        self.used += rgb_path.stat().st_size + depth_path.stat().st_size + 12000
        self.count += 1
        return True

    def close(self):
        self.records.close()
        self.labels.close()
