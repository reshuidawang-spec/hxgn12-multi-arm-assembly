#!/usr/bin/env python3
"""Preprocess the real control-cabinet STL parts for the 8-robot scene.

The SolidWorks exports in 新建文件夹/ are per-part binary STLs that share the
ASSEMBLY coordinate frame (mm).  This script:

    1. selects the 14 parts of the minimal assembly unit (dropping duplicate
       instances and leftover hardware);
    2. decimates the heavy meshes with numpy-only vertex clustering;
    3. scales by 0.0005 and centres the cabinet with its back on the pallet
       and its door opening facing world +Z;
    4. writes processed ASCII-named STLs to models/cabinet/processed/ plus a
       manifest.json used by the scene builder for pose assertions.

Usage:  python3 scripts/preprocess_cabinet_models.py
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = REPO_ROOT / "新建文件夹"
OUTPUT_DIR = REPO_ROOT / "models" / "cabinet" / "processed"

# Assembly frame (mm): X = width (905), Y = REAL HEIGHT (481), Z = depth
# (284). The solid back is at CAD z-max, open rim at z-min.
# Rx(180 degrees) preserves handedness and places the genuine opening up.
ASM_CENTER_X = 457.0
ASM_CENTER_Y = 243.6
ASM_TOP_Z = 287.2
ASM_BOTTOM_Z = 3.2
SCALE = 0.0005

# part_id -> (source substrings, triangle budget, grayscale colour)
# Every substring must appear in the source filename (part number and
# instance suffix are separated by the Chinese part name).
PARTS = {
    "shell": (("2018090800227723",), 30000, [0.66, 0.66, 0.66]),
    # Internal mounting panel adjacent to the solid back; preinstalled in
    # the shell supplied to R1. Not a door or a final closure operation.
    "mounting_panel": (("2018081800220728",), 20000, [0.78, 0.78, 0.78]),
    "rail_v1": (("2018061400196431", "-2.STL"), 2000, [0.50, 0.50, 0.50]),
    "rail_v2": (("2018061400196431", "-3.STL"), 2000, [0.50, 0.50, 0.50]),
    "rail_h1": (("2018061400196432", "-3.STL"), 2000, [0.50, 0.50, 0.50]),
    "plc": (("506H001444", "-1.STL"), 30000, [0.30, 0.30, 0.32]),
    "psu": (("103H000502",), 10000, [0.55, 0.55, 0.55]),
    "servo": (("101H000151", "-13.STL"), 6000, [0.62, 0.62, 0.64]),
    "dma": (("101H000429",), 25000, [0.42, 0.42, 0.44]),
    "contactor": (("111H000518",), 12000, [0.72, 0.72, 0.70]),
    "breaker": (("111H000520",), 5000, [0.82, 0.82, 0.82]),
    "filter": (("110H000008", "-2.STL"), 5000, [0.68, 0.68, 0.70]),
    "com5": (("111H000851",), 4000, [0.88, 0.88, 0.86]),
    "eds": (("198H001584",), 1000, [0.75, 0.75, 0.78]),
}


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (vertices Nx9, normals Nx3) for a binary STL."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"endsolid" in data.lower():
        raise ValueError(f"ASCII STL not supported: {path}")
    count = struct.unpack("<I", data[80:84])[0]
    arr = np.frombuffer(
        data[84:],
        dtype=np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
    )[:count]
    return arr["v"].astype(np.float64).reshape(-1, 9), arr["normal"].astype(np.float64)


def decimate(verts: np.ndarray, budget: int) -> tuple[np.ndarray, float]:
    """Vertex clustering; returns (vertices, cell size mm).

    Binary-searches the smallest grid cell whose clustering keeps the
    triangle count within ``budget`` (smaller cell = more detail).
    """
    n_tri = len(verts)
    if n_tri <= budget:
        return verts, 0.0

    def cluster(cell: float) -> np.ndarray:
        keys = np.floor(verts / cell).astype(np.int64)
        # unique vertex cells -> centroid
        flat = verts.reshape(-1, 3)
        flat_keys = keys.reshape(-1, 3)
        order = np.lexsort((flat_keys[:, 2], flat_keys[:, 1], flat_keys[:, 0]))
        sorted_keys = flat_keys[order]
        boundaries = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
        starts = np.concatenate(([0], np.flatnonzero(boundaries) + 1))
        centroids = np.empty((len(starts), 3))
        for idx, start in enumerate(starts):
            end = starts[idx + 1] if idx + 1 < len(starts) else len(order)
            centroids[idx] = flat[order[start:end]].mean(axis=0)
        # map each flattened vertex to its centroid
        inverse = np.empty(len(order), dtype=np.int64)
        inverse[order] = np.repeat(np.arange(len(starts)), np.diff(np.concatenate((starts, [len(order)]))))
        mapped = centroids[inverse].reshape(-1, 9)
        # drop degenerate triangles (any two corners merged)
        a, b, c = mapped[:, 0:3], mapped[:, 3:6], mapped[:, 6:9]
        keep = ~(np.all(a == b, axis=1) | np.all(a == c, axis=1) | np.all(b == c, axis=1))
        return mapped[keep]

    low, high = 0.05, 10.0
    best: tuple[np.ndarray, float] = (verts, 0.0)
    for _ in range(24):
        mid = (low + high) / 2.0
        candidate = cluster(mid)
        if len(candidate) <= budget:
            best = (candidate, mid)
            high = mid
        else:
            low = mid
    return best


def transform(verts_mm: np.ndarray) -> np.ndarray:
    """CAD mm -> metres, back down and door opening up (proper rotation)."""
    x = (verts_mm[:, 0::3] - ASM_CENTER_X) * SCALE
    y = (ASM_CENTER_Y - verts_mm[:, 1::3]) * SCALE
    z = (ASM_TOP_Z - verts_mm[:, 2::3]) * SCALE
    out = np.empty_like(verts_mm)
    out[:, 0::3] = x
    out[:, 1::3] = y
    out[:, 2::3] = z
    return out


def face_normals(verts: np.ndarray) -> np.ndarray:
    a, b, c = verts[:, 0:3], verts[:, 3:6], verts[:, 6:9]
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n, axis=1)
    norm[norm == 0] = 1.0
    return n / norm[:, None]


def write_stl(path: Path, verts: np.ndarray) -> None:
    normals = face_normals(verts)
    count = len(verts)
    with path.open("wb") as handle:
        handle.write(b"\0" * 80)
        handle.write(struct.pack("<I", count))
        for tri, normal in zip(verts, normals):
            handle.write(normal.astype("<f4").tobytes())
            handle.write(tri.astype("<f4").tobytes())
            handle.write(struct.pack("<H", 0))


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    sources = sorted(SOURCE_DIR.glob("*.STL"))
    manifest: dict = {"frame": "cabinet_open_up_v2", "parts": {}}
    for part_id, (substrings, budget, color) in PARTS.items():
        matches = [
            path
            for path in sources
            if all(substring in path.name for substring in substrings)
        ]
        if len(matches) != 1:
            raise RuntimeError(f"{part_id}: expected 1 source, found {len(matches)}")
        source = matches[0]
        verts_mm, _ = read_stl(source)
        tris_in = len(verts_mm)
        verts, cell = decimate(verts_mm, budget)
        local = transform(verts)
        write_stl(OUTPUT_DIR / f"{part_id}.stl", local)
        flat = local.reshape(-1, 3)
        lo, hi = flat.min(axis=0), flat.max(axis=0)
        manifest["parts"][part_id] = {
            "source": source.name,
            "tris_in": tris_in,
            "tris_out": len(local),
            "cell_mm": round(cell, 3),
            "bbox_lo": [round(float(v), 4) for v in lo],
            "bbox_hi": [round(float(v), 4) for v in hi],
            "color": color,
        }
        print(
            f"{part_id:10s} tris {tris_in:6d} -> {len(local):6d} "
            f"(cell {cell:.2f}mm)  bbox "
            f"x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
            f"z[{lo[2]:+.3f},{hi[2]:+.3f}] m"
        )
    (OUTPUT_DIR / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"manifest written to {OUTPUT_DIR / 'manifest.json'}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(main())
