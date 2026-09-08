"""Machine-readable contract for simple vertical pick-and-place planning."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POLICY_PATH = REPO_ROOT / "configs" / "motion_planning_policy.yaml"

EXPECTED_CYCLE = (
    "PARK", "SOURCE_HIGH", "PICK_APP", "PICK_TCP", "GRIP",
    "PICK_APP", "SOURCE_HIGH", "TARGET_HIGH", "PLACE_APP",
    "PLACE_TCP", "RELEASE", "PLACE_APP", "READY_OR_PARK",
)
EXPECTED_FALLBACK_ORDER = (
    "direct_vertical_pi", "raise_safe_height", "one_side_high_waypoint",
    "alternate_ik_branch", "constrained_ompl",
)


def load_motion_policy(path: Path = DEFAULT_POLICY_PATH) -> dict[str, Any]:
    raw = path.read_bytes()
    policy = yaml.safe_load(raw)
    if not isinstance(policy, dict):
        raise RuntimeError(f"motion policy must be a mapping: {path}")
    if tuple(policy.get("canonical_cycle", ())) != EXPECTED_CYCLE:
        raise RuntimeError("motion policy canonical cycle is not the vertical Pi cycle")
    if tuple(policy.get("fallback_order", ())) != EXPECTED_FALLBACK_ORDER:
        raise RuntimeError("motion policy fallback order was changed or reordered")
    orientation = policy.get("orientation", {})
    if orientation.get("physical_tool_axis") != "world_negative_z":
        raise RuntimeError("motion policy must point the physical tool axis toward world -Z")
    if orientation.get("yaw_change") != "safe_height_only":
        raise RuntimeError("motion policy may change yaw only at safe height")
    coordination = policy.get("coordination", {})
    if coordination.get("shared_workspace_entry") != "single_robot_exclusive":
        raise RuntimeError("shared workspaces must be single-robot exclusive")
    contact = policy.get("contact_exclusions", {})
    if not contact.get("robot_links_never_waived", False):
        raise RuntimeError("motion policy may never waive robot-link collisions")
    policy["fingerprint"] = {
        "file": str(path),
        "size": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }
    return policy


def safe_height_candidates(
    preferred_z: float,
    minimum_z: float,
    maximum_z: float,
    increment: float,
) -> list[float]:
    """Return direct height first, then monotonically raised fallbacks."""
    if increment <= 0.0:
        raise ValueError("safe-height increment must be positive")
    first = max(float(preferred_z), float(minimum_z))
    maximum = float(maximum_z)
    if first > maximum + 1e-9:
        raise ValueError("required safe height exceeds configured maximum")
    result: list[float] = []
    value = first
    while value <= maximum + 1e-9:
        result.append(round(value, 6))
        value += increment
    if result[-1] < maximum - 1e-9:
        result.append(round(maximum, 6))
    return result


def preferred_workspace_height(policy: dict[str, Any], workspace: str) -> float:
    heights = policy["clearance"]["workspace_preferred_tcp_z_m"]
    if workspace not in heights:
        raise KeyError(f"workspace has no preferred safe height: {workspace}")
    return float(heights[workspace])

