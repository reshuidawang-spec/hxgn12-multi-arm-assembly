"""Automatic assembly-process decomposition from an assembled cabinet model.

Step 1 of the automated line-planning system.  Given a set of per-part STL
files that share ONE assembly coordinate frame (the SolidWorks per-part
export convention), this module:

    1. parses every part (bbox, centre, extents);
    2. builds a contact graph (bbox proximity);
    3. infers assembly constraints with simple heuristics:
       - SUPPORT: part A's top face carries part B's bottom face
         (B.z_min within a tolerance of A.z_max, XY footprints overlap);
       - CONTAIN: part A's bbox contains part B's bbox (shell/devices);
    4. topological-sorts the constraints into an assembly-order DAG
       (height-ascending as the tie-break);
    5. maps every step to a process type and a suggested tool / robot
       using the 8-robot line's capability table;
    6. emits ``process_chain.json``.

No CoppeliaSim dependency: the module is pure numpy and can be tested
offline.  The scene-application side lives in import_cabinet_ui.py.

Usage:
    python3 test/decompose_assembly.py <folder-with-stls> [--out process_chain.json]
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Optional

import numpy as np

# ---------------------------------------------------------------------------
# STL parsing (binary + ASCII)
# ---------------------------------------------------------------------------


def read_stl(path: Path) -> np.ndarray:
    """Return vertices as an Nx9 array (3 corners per triangle row)."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"endsolid" in data.lower():
        text = data.decode("utf-8", errors="ignore")
        triangles = []
        for facet in text.split("facet normal")[1:]:
            block = facet.split("outer loop")
            if len(block) < 2:
                continue
            vertices = []
            for line in block[1].split("vertex")[1:4]:
                vertices.extend(float(value) for value in line.split()[:3])
            if len(vertices) == 9:
                triangles.append(vertices)
        return np.array(triangles, dtype=float).reshape(-1, 9)
    count = struct.unpack("<I", data[80:84])[0]
    arr = np.frombuffer(
        data[84:],
        dtype=np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
    )[:count]
    return arr["v"].astype(np.float64).reshape(-1, 9)


# ---------------------------------------------------------------------------
# robot capability table (mirrors the current 8-robot line)
# ---------------------------------------------------------------------------

ROBOT_TABLE = {
    "R1": {"tool": "gripper", "station": "conveyor/wb1", "roles": ["base_feed", "large_part"]},
    "R2": {"tool": "vacuum", "station": "stand/wb1", "roles": ["flat_panel", "thin_part"]},
    "R3": {"tool": "gripper", "station": "rack/wb1", "roles": ["thin_part", "transfer"]},
    "R4": {"tool": "gripper", "station": "handoff/wb2", "roles": ["transfer", "large_part"]},
    "R5": {"tool": "vacuum", "station": "basket/wb2", "roles": ["flat_panel", "device"]},
    "R6": {"tool": "gripper", "station": "basket/wb2", "roles": ["device", "transfer"]},
    "R7": {"tool": "screwdriver", "station": "staging", "roles": ["fastener"]},
    "R8": {"tool": "gripper", "station": "staging/output", "roles": ["transfer", "sort"]},
}

SCREW_KEYWORDS = ("screw", "bolt", "螺栓", "螺钉", "螺丝")
PLATE_KEYWORDS = ("panel", "plate", "door", "cover", "板", "门", "盖")
RAIL_KEYWORDS = ("rail", "truss", "bar", "导轨", "桁架", "轨")
FRAME_KEYWORDS = ("梁", "beam", "frame", "柱", "column")
BRACKET_KEYWORDS = ("支架", "支板", "夹板", "联接", "bracket", "support", "clamp", "轴", "shaft")
DEVICE_KEYWORDS = ("plc", "power", "supply", "drive", "contactor", "breaker",
                   "filter", "socket", "switch", "module", "电源", "驱动",
                   "接触器", "断路器", "滤波器", "模块", "开关", "绝缘子",
                   "互感", "绝缘", "接地", "断路器", "开关")
SHELL_KEYWORDS = ("shell", "cabinet", "box", "body", "壳体", "柜体", "箱体", "外壳", "本体")


class Part:
    def __init__(self, name: str, path: Path):
        self.name = name
        self.path = path
        vertices = read_stl(path)
        flat = vertices.reshape(-1, 3)
        self.bbox_lo = flat.min(axis=0)
        self.bbox_hi = flat.max(axis=0)
        self.center = (self.bbox_lo + self.bbox_hi) / 2.0
        self.extents = self.bbox_hi - self.bbox_lo
        self.triangle_count = len(vertices)

    def footprint_overlaps(self, other: "Part", margin: float = 0.005) -> bool:
        for axis in (0, 1):
            if (
                self.bbox_hi[axis] + margin < other.bbox_lo[axis]
                or other.bbox_hi[axis] + margin < self.bbox_lo[axis]
            ):
                return False
        return True

    def contains(self, other: "Part", margin: float = 0.002) -> bool:
        return all(
            self.bbox_lo[axis] - margin <= other.bbox_lo[axis]
            and other.bbox_hi[axis] <= self.bbox_hi[axis] + margin
            for axis in range(3)
        )

    def supports(self, other: "Part", tolerance: float = 0.008) -> bool:
        """self carries other: other's bottom rests on self's top."""
        if not self.footprint_overlaps(other):
            return False
        return abs(other.bbox_lo[2] - self.bbox_hi[2]) <= tolerance

    def classify(self) -> dict:
        name_lower = self.name.lower()
        longest, thinnest = 0, 0
        for axis in range(3):
            if self.extents[axis] > self.extents[longest]:
                longest = axis
            if self.extents[axis] < self.extents[thinnest]:
                thinnest = axis
        flatness = self.extents[thinnest] / max(self.extents[longest], 1e-9)
        size = float(np.prod(self.extents))
        if any(keyword in name_lower for keyword in SHELL_KEYWORDS):
            return {"process": "shell_feed", "role": "base_feed"}
        if any(keyword in name_lower for keyword in SCREW_KEYWORDS):
            return {"process": "screw", "role": "fastener"}
        # long members first: they are the frame the rest attaches to
        if any(keyword in name_lower for keyword in FRAME_KEYWORDS):
            return {"process": "frame_install", "role": "frame_member"}
        if any(keyword in name_lower for keyword in BRACKET_KEYWORDS):
            return {"process": "bracket_install", "role": "bracket"}
        if any(keyword in name_lower for keyword in RAIL_KEYWORDS):
            return {"process": "rail_install", "role": "thin_part"}
        if any(keyword in name_lower for keyword in DEVICE_KEYWORDS):
            return {"process": "device_install", "role": "device"}
        if flatness < 0.08 and self.extents[longest] > 0.15:
            return {"process": "panel_install", "role": "flat_panel"}
        if any(keyword in name_lower for keyword in PLATE_KEYWORDS):
            return {"process": "panel_install", "role": "flat_panel"}
        if size > 0.004:
            return {"process": "device_install", "role": "device"}
        return {"process": "small_part_install", "role": "thin_part"}


def assign_robot(role: str, part: Part) -> tuple[str, str]:
    """Pick the best robot for a role (simple capability match)."""
    preferred = {
        "base_feed": ("R1", "conveyor"),
        "frame_member": ("R3", "rack"),
        "flat_panel": ("R2", "stand"),
        "thin_part": ("R3", "rack"),
        "bracket": ("R6", "basket"),
        "device": ("R6", "basket"),
        "fastener": ("R7", "staging"),
        "transfer": ("R4", "handoff"),
    }
    robot_id, station = preferred.get(role, ("R6", "basket"))
    return robot_id, station


# assembly layer priority: frame first, fasteners last
ROLE_PRIORITY = {
    "base_feed": 0,
    "frame_member": 1,
    "flat_panel": 2,
    "thin_part": 3,
    "bracket": 4,
    "device": 5,
    "fastener": 6,
}


def _strip_instance_suffix(name: str) -> str:
    """'大梁2000-3' -> '大梁2000' (keep non-numeric prefixes intact)."""
    if "-" not in name:
        return name
    prefix, suffix = name.rsplit("-", 1)
    if suffix.strip().isdigit():
        return prefix
    return name


def group_by_geometry(parts: list[Part]) -> dict[str, dict]:
    """Merge duplicate instances (same file size + same extents).

    Returns {representative_name: {"part": Part, "instances": int}}.
    """
    groups: dict[tuple, list[Part]] = {}
    for part in parts:
        key = (
            part.path.stat().st_size,
            tuple(round(float(value), 4) for value in part.extents),
        )
        groups.setdefault(key, []).append(part)
    merged: dict[str, dict] = {}
    for members in groups.values():
        representative = _strip_instance_suffix(members[0].name)
        members[0].name = representative  # order/constraints use this name
        merged[representative] = {
            "part": members[0],
            "instances": len(members),
        }
    return merged


# ---------------------------------------------------------------------------
# decomposition
# ---------------------------------------------------------------------------


def build_contact_graph(parts: list[Part], tolerance: float = 0.006) -> dict:
    edges = []
    for first in range(len(parts)):
        for second in range(first + 1, len(parts)):
            a, b = parts[first], parts[second]
            gap = max(
                0.0,
                max(a.bbox_lo - b.bbox_hi).max(),
                max(b.bbox_lo - a.bbox_hi).max(),
            )
            if gap <= tolerance:
                edges.append((a.name, b.name, round(float(gap), 5)))
    return {"tolerance_m": tolerance, "edges": edges}


def build_constraints(parts: list[Part]) -> dict:
    """Support and containment constraints as (before, after) pairs."""
    supports: list[tuple[str, str]] = []
    contains: list[tuple[str, str]] = []
    for carrier in parts:
        for other in parts:
            if carrier is other:
                continue
            if carrier.supports(other):
                supports.append((carrier.name, other.name))
            if carrier.contains(other) and carrier is not other:
                contains.append((carrier.name, other.name))
    # deduplicate while keeping order
    supports = list(dict.fromkeys(supports))
    contains = list(dict.fromkeys(contains))
    return {"supports": supports, "contains": contains}


def topological_order(
    parts: list[Part],
    supports: list[tuple[str, str]],
    contains: list[tuple[str, str]],
) -> list[str]:
    """Order parts so supporters/containers precede their dependents.

    Ties break by assembly-layer priority (frame first), then height.
    """
    by_name = {part.name: part for part in parts}
    remaining = set(by_name)
    order: list[str] = []

    def ready(name: str) -> bool:
        for before, after in supports + contains:
            if after == name and before in remaining:
                return False
        return True

    def sort_key(name: str) -> tuple:
        part = by_name[name]
        role = part.classify()["role"]
        return (
            ROLE_PRIORITY.get(role, 7),
            float(part.center[2]),
            name,
        )

    while remaining:
        candidates = [name for name in remaining if ready(name)]
        if not candidates:
            # constraint cycle: fall back to priority order for the rest
            candidates = list(remaining)
        candidates.sort(key=sort_key)
        chosen = candidates[0]
        remaining.remove(chosen)
        order.append(chosen)
    return order


def decompose(folder: Path) -> dict:
    """Run the full decomposition on a folder of per-part STLs.

    Duplicate instances (same geometry, different positions) are merged
    into one process step with an instance count.
    """
    paths = sorted(folder.glob("*.STL")) + sorted(folder.glob("*.stl"))
    if not paths:
        raise RuntimeError(f"no STL files found in {folder}")
    raw_parts = [Part(path.stem, path) for path in paths]
    groups = group_by_geometry(raw_parts)
    parts = [entry["part"] for entry in groups.values()]
    graph = build_contact_graph(parts)
    constraints = build_constraints(parts)
    order = topological_order(parts, constraints["supports"], constraints["contains"])

    by_name = {p.name: p for p in parts}
    steps = []
    for index, name in enumerate(order, start=1):
        part = by_name[name]
        info = part.classify()
        robot_id, station = assign_robot(info["role"], part)
        steps.append({
            "index": index,
            "part": name,
            "instances": groups[name]["instances"],
            "process": info["process"],
            "role": info["role"],
            "suggested_robot": robot_id,
            "suggested_station": station,
            "center_m": [round(float(v), 4) for v in part.center],
            "extents_m": [round(float(v), 4) for v in part.extents],
            "triangles": part.triangle_count,
        })

    return {
        "schema_version": 2,
        "source_folder": str(folder),
        "file_count": len(raw_parts),
        "part_count": len(parts),
        "contact_graph": graph,
        "constraints": constraints,
        "assembly_order": order,
        "process_chain": steps,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, help="folder with per-part STLs")
    parser.add_argument("--out", type=Path, default=Path("process_chain.json"))
    args = parser.parse_args()
    result = decompose(args.folder)
    args.out.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"parts: {result['part_count']}")
    print(f"contact edges: {len(result['contact_graph']['edges'])}")
    print("assembly order:")
    for step in result["process_chain"]:
        print(
            f"  {step['index']:2d}. {step['part']:<35s} "
            f"{step['process']:<18s} -> {step['suggested_robot']} "
            f"({step['suggested_station']})"
        )
    print(f"process chain written to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
