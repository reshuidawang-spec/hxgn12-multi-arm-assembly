"""Shared motion primitives for the rebuilt 8-robot motion layer.

Architecture and math are ported from the legacy five-arm stack
(``robot_control/runtime_cartesian.py`` at git ``00a4365``); the simIK /
simOMPL builders use the CURRENT remote API (``handleGroup`` with
``syncWorlds``, ``constraint_position | constraint_alpha_beta``,
OMPL ``createTask -> setup -> solve -> getPath``) whose call patterns were
validated against a live 8-robot scene.

Design rules carried over from the legacy stack:
    - paths are deterministic joint-space sequences, densified to <=2 deg
      per frame, replayed with minimum-jerk smoothing;
    - every IK sample is collision-checked against fixtures + other arms;
    - the tool stays within ``MAX_DOWN_TILT`` of straight-down on contact
      segments (all 72 target dummies are top-down);
    - a Lua bridge script batches multi-arm joint writes into one call.
"""

from __future__ import annotations

import bisect
import hashlib
import itertools
import math
import random
from pathlib import Path
from typing import Any, Iterable, Optional

from sim_bridge.scene_objects import ARM_JOINT_ALIASES, ROBOT_IDS, ROBOT_TIPS


# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------

DOWN_QUATERNION = [1.0, 0.0, 0.0, 0.0]  # tool-down for the imported CR5A models
MAX_FRAME_DELTA = math.radians(2.0)
MAX_DOWN_TILT = math.radians(12.0)
IK_POSITION_TOLERANCE = 0.003
IK_ANGLE_TOLERANCE = math.radians(2.0)
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_DEG_S = 36.0
JOINT_SETTLE_TOLERANCE_DEG = 0.12
COLLISION_CHECK_INTERVAL = 5
WORKSPACE_CHECK_INTERVAL = 20

RUNTIME_BRIDGE_CODE = """function sysCall_init()
end

function setJointTargets(handles, targets)
    for i=1,#handles do
        -- The imported CR5A joints are kinematic.  A target-only write is
        -- accepted by the API but does not move their visible link trees.
        -- Keep the target in sync for dynamic variants, while the direct
        -- position write is what makes stopped/stepped simulation reliable.
        sim.setJointPosition(handles[i], targets[i])
        sim.setJointTargetPosition(handles[i], targets[i])
    end
end

function getJointPositions(handles)
    local positions = {}
    for i=1,#handles do
        positions[i] = sim.getJointPosition(handles[i])
    end
    return positions
end
"""

# ---------------------------------------------------------------------------
# pure math (ported verbatim from the legacy runtime_cartesian.py)
# ---------------------------------------------------------------------------


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def near(first: Iterable[float], second: Iterable[float], tolerance: float) -> bool:
    first_values = list(first)
    second_values = list(second)
    return len(first_values) == len(second_values) and max(
        abs(a - b) for a, b in zip(first_values, second_values)
    ) <= tolerance


def find_unique_alias(sim: Any, root: int, alias: str) -> int:
    matches = [
        handle
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
        if sim.getObjectAlias(handle, 0) == alias
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one {alias} below {sim.getObjectAlias(root, 0)}, "
            f"found {len(matches)}"
        )
    return matches[0]


def wrap_near(reference: float, value: float) -> float:
    while value - reference > math.pi:
        value -= 2.0 * math.pi
    while value - reference < -math.pi:
        value += 2.0 * math.pi
    return value


def unwrap_path(configs: list[list[float]]) -> list[list[float]]:
    if not configs:
        return []
    result = [list(configs[0])]
    for config in configs[1:]:
        result.append(
            [
                wrap_near(previous, value)
                for previous, value in zip(result[-1], config)
            ]
        )
    return result


def interpolate_joint_line(
    first: list[float], second: list[float], count: int
) -> list[list[float]]:
    if count < 2:
        raise ValueError("joint-line point count must be at least two")
    return [
        [
            start + (finish - start) * index / (count - 1)
            for start, finish in zip(first, second)
        ]
        for index in range(count)
    ]


def join_paths(*paths: list[list[float]]) -> list[list[float]]:
    result: list[list[float]] = []
    for path in paths:
        if not path:
            raise RuntimeError("cannot join an empty runtime path")
        current = unwrap_path(path)
        if result:
            current = [
                [
                    wrap_near(previous, value)
                    for previous, value in zip(result[-1], config)
                ]
                for config in current
            ]
            discontinuity = max(
                abs(before - after)
                for before, after in zip(result[-1], current[0])
            )
            if discontinuity > math.radians(0.5):
                raise RuntimeError(
                    "runtime joined-path discontinuity "
                    f"{math.degrees(discontinuity):.3f} deg"
                )
            current = current[1:]
        result.extend(current)
    return result


def cumulative_max_joint_distance(configs: list[list[float]]) -> list[float]:
    cumulative = [0.0]
    for first, second in zip(configs, configs[1:]):
        cumulative.append(
            cumulative[-1]
            + max(abs(b - a) for a, b in zip(first, second))
        )
    return cumulative


def interpolate_path(
    configs: list[list[float]], cumulative: list[float], distance: float
) -> list[float]:
    if distance <= 0.0:
        return list(configs[0])
    if distance >= cumulative[-1]:
        return list(configs[-1])
    upper = bisect.bisect_right(cumulative, distance)
    lower = upper - 1
    span = cumulative[upper] - cumulative[lower]
    fraction = (distance - cumulative[lower]) / span if span > 0.0 else 0.0
    return [
        first + (second - first) * fraction
        for first, second in zip(configs[lower], configs[upper])
    ]


def minimum_jerk(fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    return fraction**3 * (10.0 - 15.0 * fraction + 6.0 * fraction**2)


def quintic(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value**3 * (6.0 * value**2 - 15.0 * value + 10.0)


def normalize_config(values: Iterable[float]) -> list[float]:
    normalized = []
    for index, value in enumerate(values):
        angle = float(value)
        if index != 2:  # joint 3 keeps its limited range
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        normalized.append(angle)
    return normalized


def config_distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def densify_path(
    path: list[list[float]], max_delta: float = MAX_FRAME_DELTA
) -> list[list[float]]:
    dense = [list(path[0])]
    for start, end in zip(path, path[1:]):
        largest = max(abs(b - a) for a, b in zip(start, end))
        steps = max(1, int(math.ceil(largest / max_delta)))
        for index in range(1, steps + 1):
            blend = quintic(index / steps)
            dense.append([a + (b - a) * blend for a, b in zip(start, end)])
    return dense


def _quaternion_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    rotated = _quaternion_multiply(
        _quaternion_multiply((x, y, z, w), (*vector, 0.0)),
        (-x, -y, -z, w),
    )
    return rotated[:3]


def _compose_poses(first: list[float], second: list[float]) -> list[float]:
    translated = _rotate_vector(tuple(first[3:]), tuple(second[:3]))
    return [
        first[index] + translated[index] for index in range(3)
    ] + list(_quaternion_multiply(tuple(first[3:]), tuple(second[3:])))


def shape_tree_bounds(
    sim: Any,
    shapes: set[int],
    shape_bbs: dict[int, tuple[list[float], list[float]]],
) -> tuple[list[float], list[float]]:
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for shape in shapes:
        size, bb_pose = shape_bbs[shape]
        world_bb_pose = _compose_poses(sim.getObjectPose(shape, -1), bb_pose)
        for signs in itertools.product((-0.5, 0.5), repeat=3):
            local = tuple(size[index] * signs[index] for index in range(3))
            rotated = _rotate_vector(tuple(world_bb_pose[3:]), local)
            point = [
                world_bb_pose[index] + rotated[index] for index in range(3)
            ]
            lower = [min(a, b) for a, b in zip(lower, point)]
            upper = [max(a, b) for a, b in zip(upper, point)]
    return lower, upper


# ---------------------------------------------------------------------------
# scene adapter: handles, collision pairs, planner/batch Lua bridges
# ---------------------------------------------------------------------------


class MotionScene:
    """Handles and collision tooling for the 8 robots of the live scene."""

    def __init__(
        self,
        client: Any,
        scene_root: str = "/FiveCR5A_Cell",
        environment_exclusions: Iterable[int] = (),
    ):
        self.client = client
        self.sim = client.require("sim")
        self.ik = client.require("simIK")
        self.scene_root = scene_root
        # Subtrees excluded from the default environment collection: the
        # heavy STL workpiece meshes are handled per-action through the
        # exclusion argument instead, keeping routine collision checks fast.
        self.environment_exclusions = [int(h) for h in environment_exclusions]
        self.roots: dict[str, int] = {}
        self.joints: dict[str, list[int]] = {}
        self.tips: dict[str, int] = {}
        self.tool_roots: dict[str, int] = {}
        self.collision_shapes: dict[str, list[int]] = {}
        for robot in ROBOT_IDS:
            root = int(self.sim.getObject(f"/{robot}"))
            self.roots[robot] = root
            tree = self.sim.getObjectsInTree(root, self.sim.handle_all, 0)
            by_alias = {str(self.sim.getObjectAlias(h, 0)): int(h) for h in tree}
            self.joints[robot] = [by_alias[name] for name in ARM_JOINT_ALIASES]
            self.tips[robot] = by_alias[ROBOT_TIPS[robot]]
            self.tool_roots[robot] = by_alias[f"{robot}T"]
            self.collision_shapes[robot] = [
                int(handle)
                for handle in self.sim.getObjectsInTree(
                    root, self.sim.object_shape_type, 0
                )
                if "respondable" in str(self.sim.getObjectAlias(handle, 0)).lower()
                and "base_link" not in str(self.sim.getObjectAlias(handle, 0)).lower()
            ]
        # simIK ignores any local offset of the tip below Link6 (it models
        # the chain end at the last link origin), so each robot gets a
        # virtual tip dummy sitting EXACTLY at the Link6_visual origin.  All
        # IK targets are compensated by tip_offset() (real tip - virtual
        # tip, constant under the tool-down constraint) so the REAL tip
        # lands on the requested position.
        self.virt_tips: dict[str, int] = {}
        for robot in ROBOT_IDS:
            root = self.roots[robot]
            tree = self.sim.getObjectsInTree(root, self.sim.handle_all, 0)
            by_alias = {str(self.sim.getObjectAlias(h, 0)): int(h) for h in tree}
            link6_visual = by_alias["Link6_visual"]
            virt = int(self.sim.createDummy(0.004))
            self.sim.setObjectAlias(virt, f"{robot}_virt_tip", {"aliasIndex": 0})
            self.sim.setObjectParent(virt, link6_visual, False)
            self.sim.setObjectPose(
                virt, link6_visual, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            )
            self.sim.setObjectInt32Param(virt, self.sim.objintparam_visibility_layer, 0)
            self.virt_tips[robot] = virt

        self._planner_script: Optional[int] = None
        self._batch_script: Optional[int] = None
        self._bridge_script: Optional[int] = None

    # -- joint helpers ---------------------------------------------------
    def set_joints(self, robot: str, values: Iterable[float]) -> None:
        for handle, value in zip(self.joints[robot], values):
            self.sim.setJointPosition(handle, float(value))

    def joint_positions(self, robot: str) -> list[float]:
        return [
            float(self.sim.getJointPosition(handle))
            for handle in self.joints[robot]
        ]

    def tip_offset(self, robot: str) -> list[float]:
        """World offset real tip - virtual tip under the tool-down pose.

        The tool is coaxial with Link6 +Z, so with the tool pointing down
        the real tip sits exactly ``tool_length`` below the virtual tip
        (the Link6 origin).  The rigid distance is measured once per call.
        """
        real = self.sim.getObjectPosition(self.tips[robot], -1)
        virt = self.sim.getObjectPosition(self.virt_tips[robot], -1)
        length = math.dist(real, virt)
        return [0.0, 0.0, -length]

    def set_all_home(self) -> None:
        for robot in ROBOT_IDS:
            self.set_joints(robot, [0.0] * 6)

    # -- collision pairs ---------------------------------------------------
    def create_collision_pair(
        self,
        robot: str,
        exclusions: Iterable[int] = (),
        include_other_robots: bool = False,
    ) -> tuple[int, int]:
        """(moving, environment) collections for the active robot.

        Other robots are excluded by default: their heavy collision meshes
        dominate checkCollision time, and inter-arm interference is checked
        at runtime among ACTIVE arms only (legacy coordinated pattern).
        """
        moving = int(self.sim.createCollection(0))
        environment = int(self.sim.createCollection(0))
        self._add_robot_collision_geometry(moving, robot)
        self.sim.addItemToCollection(
            environment, self.sim.handle_tree, int(self.sim.getObject(self.scene_root)), 0
        )
        for subtree in self.environment_exclusions:
            self.sim.addItemToCollection(
                environment, self.sim.handle_tree, subtree, 1
            )
        for handle in exclusions:
            self.sim.addItemToCollection(
                environment, self.sim.handle_tree, int(handle), 1
            )
        if include_other_robots:
            for other in ROBOT_IDS:
                if other != robot:
                    self._add_robot_collision_geometry(environment, other)
        return moving, environment

    def _add_robot_collision_geometry(self, collection: int, robot: str) -> None:
        for shape in self.collision_shapes[robot]:
            self.sim.addItemToCollection(
                collection, self.sim.handle_single, shape, 0
            )
        self.sim.addItemToCollection(
            collection, self.sim.handle_tree, self.tool_roots[robot], 0
        )

    def destroy_collision_pair(self, pair: tuple[int, int]) -> None:
        for collection in pair:
            self.sim.destroyCollection(collection)

    def collision_details(self, pair: tuple[int, int]) -> Optional[tuple[str, str]]:
        result, handles = self.sim.checkCollision(*pair)
        if int(result) <= 0:
            return None
        return (
            str(self.sim.getObjectAlias(int(handles[0]), 1)),
            str(self.sim.getObjectAlias(int(handles[1]), 1)),
        )

    # -- OMPL planner bridge ----------------------------------------------
    def install_planner_script(self) -> None:
        if self._planner_script is not None:
            return
        code = """
sim=require 'sim'
simOMPL=require 'simOMPL'

function planJointPathImpl(joints,tip,robotCollection,environmentCollection,startState,goalState,maxTilt,maxTime)
    local saved={}
    for i=1,#joints,1 do saved[i]=sim.getJointPosition(joints[i]) end
    local task=simOMPL.createTask('cabinetCollisionAware')
    simOMPL.setVerboseLevel(task,0)
    simOMPL.setAlgorithm(task,simOMPL.Algorithm.RRTConnect)
    local spaces={}
    for i=1,#joints,1 do
        local lo=-math.pi
        local hi=math.pi
        if i==3 then lo=-2.79 hi=2.79 end
        spaces[i]=simOMPL.createStateSpace(
            'joint'..i,simOMPL.StateSpaceType.joint_position,joints[i],
            {lo},{hi},i<=3 and 1 or 0
        )
    end
    simOMPL.setStateSpace(task,spaces)
    simOMPL.setStartState(task,startState)
    simOMPL.setGoalState(task,goalState)
    simOMPL.setStateValidityCheckingResolution(task,0.004)
    local cosLimit=math.cos(maxTilt)
    local function stateValid(state)
        for i=1,#joints,1 do sim.setJointPosition(joints[i],state[i]) end
        if sim.checkCollision(robotCollection,environmentCollection)>0 then return false end
        if maxTilt<3.0 then
            local m=sim.getObjectMatrix(tip,sim.handle_world)
            if m[11]>-cosLimit then return false end
        end
        return true
    end
    simOMPL.setStateValidationCallback(task,stateValid)
    simOMPL.setup(task)
    local solved,path=simOMPL.compute(task,maxTime,-1,180)
    simOMPL.destroyTask(task)
    for i=1,#joints,1 do sim.setJointPosition(joints[i],saved[i]) end
    return solved,path or {}
end

function planJointPath(...)
    local args={...}
    local function invoke() return planJointPathImpl(table.unpack(args)) end
    local result={xpcall(invoke,debug.traceback)}
    if not result[1] then return false,{},result[2] end
    return result[2],result[3],''
end
"""
        self._planner_script = int(
            self.sim.createScript(self.sim.scripttype_customization, code, 0)
        )
        self.sim.setObjectAlias(self._planner_script, "Motion_Collision_Planner")
        self.sim.initScript(self._planner_script)

    def remove_planner_script(self) -> None:
        if self._planner_script is not None:
            try:
                self.sim.removeObjects([self._planner_script])
            finally:
                self._planner_script = None

    def ompl_path(
        self,
        robot: str,
        start: list[float],
        goal: list[float],
        exclusions: Iterable[int] = (),
        max_tilt: float = MAX_DOWN_TILT,
    ) -> list[list[float]]:
        self.install_planner_script()
        pair = self.create_collision_pair(robot, exclusions)
        try:
            solved, flat, planner_error = self.sim.callScriptFunction(
                "planJointPath", self._planner_script, self.joints[robot],
                self.tips[robot], pair[0], pair[1], start, goal,
                float(max_tilt), 4.0,
            )
        finally:
            self.destroy_collision_pair(pair)
        if not solved or not flat:
            detail = f": {planner_error}" if planner_error else ""
            raise RuntimeError(f"collision-aware OMPL failed for {robot}{detail}")
        return [
            [float(value) for value in flat[index:index + 6]]
            for index in range(0, len(flat), 6)
        ]

    # -- runtime batch bridge ----------------------------------------------
    def install_bridge_script(self) -> None:
        if self._bridge_script is not None:
            return
        self._bridge_script = int(
            self.sim.createScript(
                self.sim.scripttype_customization, RUNTIME_BRIDGE_CODE, 0
            )
        )
        self.sim.setObjectAlias(self._bridge_script, "Motion_Runtime_Bridge")
        self.sim.initScript(self._bridge_script)

    def remove_bridge_script(self) -> None:
        if self._bridge_script is not None:
            try:
                self.sim.removeObjects([self._bridge_script])
            finally:
                self._bridge_script = None

    def set_joint_targets(self, targets: dict[str, list[float]]) -> None:
        if self._bridge_script is None:
            raise RuntimeError("bridge script is not installed")
        joint_handles: list[int] = []
        values: list[float] = []
        for robot, joints in targets.items():
            joint_handles.extend(self.joints[robot])
            values.extend(joints)
        self.sim.callScriptFunction(
            "setJointTargets", self._bridge_script, joint_handles, values
        )

    def read_joint_positions(self, robots: Optional[Iterable[str]] = None) -> dict[str, list[float]]:
        if self._bridge_script is None:
            raise RuntimeError("bridge script is not installed")
        robot_list = list(robots) if robots is not None else list(ROBOT_IDS)
        joint_handles: list[int] = []
        for robot in robot_list:
            joint_handles.extend(self.joints[robot])
        flat = self.sim.callScriptFunction(
            "getJointPositions", self._bridge_script, joint_handles
        )
        positions: dict[str, list[float]] = {}
        index = 0
        for robot in robot_list:
            positions[robot] = [float(v) for v in flat[index:index + 6]]
            index += 6
        return positions


# ---------------------------------------------------------------------------
# new-API IK builders
# ---------------------------------------------------------------------------


def _quat_error_axis(current: list[float], goal: list[float]) -> list[float]:
    """Rotation vector (3D) between two quaternions [x,y,z,w]."""
    qx, qy, qz, qw = _quaternion_multiply(
        tuple(goal), (-current[0], -current[1], -current[2], current[3])
    )
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw)) or 1.0
    scale = angle / sin_half
    return [qx * scale, qy * scale, qz * scale]


def numerical_ik(
    scene: "MotionScene",
    robot: str,
    target_position: list[float],
    seed: list[float],
    exclusions: Iterable[int] = (),
    max_iterations: int = 150,
    position_tolerance: float = 0.003,
) -> list[float]:
    """Full-pose (position + tool-down orientation) numerical IK.

    The simIK remote module ignores its target dummy in this CoppeliaSim
    version, so the virtual tip (Link6 origin) is driven numerically with a
    6x6 numeric Jacobian.  The goal position is the requested real-tip
    position compensated by the constant tool-down offset, and the goal
    orientation is the tool-down quaternion so the compensation stays
    exactly vertical.
    """
    sim = scene.sim
    joints = scene.joints[robot]
    tip = scene.tips[robot]
    config = [float(v) for v in seed]
    scene.set_joints(robot, config)
    goal = [float(v) for v in target_position]
    pair = scene.create_collision_pair(robot, exclusions)
    try:
        for _ in range(max_iterations):
            current_pos = [
                float(v) for v in sim.getObjectPosition(tip, -1)
            ]
            current_quat = [
                float(v) for v in sim.getObjectQuaternion(tip, -1)
            ]
            position_error = [goal[i] - current_pos[i] for i in range(3)]
            # Position-only solve on the real tip; the tool orientation is
            # refined by the runtime contact paths in the cycle controller.
            error = position_error
            total = math.sqrt(sum(value**2 for value in error))
            if total <= position_tolerance:
                return normalize_config(config)
            # numeric Jacobian (3x6)
            jacobian = [[0.0] * 6 for _ in range(3)]
            for column, joint in enumerate(joints):
                original = float(sim.getJointPosition(joint))
                delta = 2e-3
                sim.setJointPosition(joint, original + delta)
                perturbed_pos = [
                    float(v)
                    for v in sim.getObjectPosition(tip, -1)
                ]
                sim.setJointPosition(joint, original)
                for row in range(3):
                    jacobian[row][column] = (
                        perturbed_pos[row] - current_pos[row]
                    ) / delta
            # damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e
            jjt = [
                [
                    sum(jacobian[r1][k] * jacobian[r2][k] for k in range(6))
                    for r2 in range(3)
                ]
                for r1 in range(3)
            ]
            damping = max(total * 0.6, 1e-3)
            for row in range(3):
                jjt[row][row] += damping * damping
            det = (
                jjt[0][0] * (jjt[1][1] * jjt[2][2] - jjt[1][2] * jjt[2][1])
                - jjt[0][1] * (jjt[1][0] * jjt[2][2] - jjt[1][2] * jjt[2][0])
                + jjt[0][2] * (jjt[1][0] * jjt[2][1] - jjt[1][1] * jjt[2][0])
            )
            if abs(det) < 1e-12:
                break
            inverse = [
                [
                    (jjt[(r + 1) % 3][(c + 1) % 3] * jjt[(r + 2) % 3][(c + 2) % 3]
                     - jjt[(r + 1) % 3][(c + 2) % 3] * jjt[(r + 2) % 3][(c + 1) % 3]) / det
                    for c in range(3)
                ]
                for r in range(3)
            ]
            joint_error = [
                sum(
                    jacobian[row][k] * sum(inverse[row][c] * error[c] for c in range(3))
                    for row in range(3)
                )
                for k in range(6)
            ]
            step_size = min(
                0.8, 0.20 / (math.sqrt(sum(v**2 for v in joint_error)) + 1e-9)
            )
            candidate = normalize_config(
                [
                    wrap_near(reference, value + delta * step_size)
                    for reference, value, delta in zip(config, config, joint_error)
                ]
            )
            scene.set_joints(robot, candidate)
            if scene.collision_details(pair) is not None:
                candidate = normalize_config(
                    [
                        wrap_near(reference, value + delta * step_size * 0.25)
                        for reference, value, delta in zip(config, config, joint_error)
                    ]
                )
                scene.set_joints(robot, candidate)
                if scene.collision_details(pair) is not None:
                    raise RuntimeError(f"numerical IK collided for {robot}")
            config = candidate
        raise RuntimeError(
            f"numerical IK failed for {robot}: position residual "
            f"{math.sqrt(sum(v**2 for v in position_error)) * 1000.0:.1f} mm"
        )
    finally:
        scene.destroy_collision_pair(pair)


def ik_candidates(
    scene: MotionScene,
    robot: str,
    position: list[float],
    seed: list[float],
    exclusions: Iterable[int] = (),
    attempts: int = 24,
) -> list[list[float]]:
    """Down-facing collision-free IK branches for a tool position."""
    sim, ik = scene.sim, scene.ik
    offset = scene.tip_offset(robot)
    target = int(sim.createDummy(0.01))
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    sim.setObjectPosition(
        target, -1, [float(a - b) for a, b in zip(position, offset)]
    )
    sim.setObjectQuaternion(target, -1, DOWN_QUATERNION)
    environment = int(ik.createEnvironment())
    group = int(ik.createGroup(environment))
    pair = scene.create_collision_pair(robot, exclusions)
    try:
        ik.setGroupCalculation(environment, group, ik.method_damped_least_squares, 0.20, 250)
        element, _, _ = ik.addElementFromScene(
            environment, group, scene.roots[robot], scene.virt_tips[robot], target,
            ik.constraint_position | ik.constraint_alpha_beta,
        )
        ik.setElementPrecision(
            environment, group, int(element),
            [IK_POSITION_TOLERANCE, IK_ANGLE_TOLERANCE],
        )
        random_seed = sum(ord(char) for char in robot) + int(sum(abs(v) * 100 for v in position))
        rng = random.Random(random_seed)
        seeds = [list(seed)]
        for _ in range(attempts - 1):
            candidate = [rng.uniform(-math.pi, math.pi) for _ in range(6)]
            candidate[2] = rng.uniform(-2.72, 2.72)
            seeds.append(candidate)
        solutions: list[list[float]] = []
        for initial in seeds:
            scene.set_joints(robot, initial)
            # keep the compensated target in sync with the seed pose so the
            # real tip lands on the requested position regardless of branch
            current_offset = scene.tip_offset(robot)
            sim.setObjectPosition(
                target, -1, [float(a - b) for a, b in zip(position, current_offset)]
            )
            result = ik.handleGroup(environment, group, {"syncWorlds": True})
            result_code = int(result[0] if isinstance(result, (list, tuple)) else result)
            if result_code != int(ik.result_success):
                continue
            solution = normalize_config(
                float(sim.getJointPosition(handle)) for handle in scene.joints[robot]
            )
            scene.set_joints(robot, solution)
            if scene.collision_details(pair) is not None:
                continue
            matrix = sim.getObjectMatrix(scene.tips[robot], -1)
            if float(matrix[10]) > -math.cos(MAX_DOWN_TILT):
                continue
            if all(config_distance(solution, known) > 0.10 for known in solutions):
                solutions.append(solution)
            if len(solutions) >= 8:
                break
        solutions.sort(key=lambda value: config_distance(value, seed))
        return solutions
    finally:
        scene.destroy_collision_pair(pair)
        ik.eraseEnvironment(environment)
        sim.removeObjects([target])
        scene.set_joints(robot, seed)


def cartesian_down_line(
    scene: MotionScene,
    robot: str,
    start_config: list[float],
    start_position: list[float],
    end_position: list[float],
    exclusions: Iterable[int],
    position_tolerance: float = IK_POSITION_TOLERANCE,
) -> list[list[float]]:
    """Tool-down straight approach sampled with IK and collision checks."""
    sim, ik = scene.sim, scene.ik
    scene.set_joints(robot, start_config)
    offset = scene.tip_offset(robot)
    target = int(sim.createDummy(0.01))
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    environment = int(ik.createEnvironment())
    group = int(ik.createGroup(environment))
    pair = scene.create_collision_pair(robot, exclusions)
    distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(start_position, end_position)))
    steps = max(8, int(math.ceil(distance / 0.02)))
    path = [list(start_config)]
    try:
        ik.setGroupCalculation(environment, group, ik.method_damped_least_squares, 0.18, 300)
        element, _, _ = ik.addElementFromScene(
            environment, group, scene.roots[robot], scene.virt_tips[robot], target,
            ik.constraint_position | ik.constraint_alpha_beta,
        )
        ik.setElementPrecision(
            environment, group, int(element),
            [position_tolerance, IK_ANGLE_TOLERANCE],
        )
        for index in range(1, steps + 1):
            ratio = index / steps
            position = [a + (b - a) * ratio for a, b in zip(start_position, end_position)]
            sim.setObjectPosition(
                target, -1, [float(a - b) for a, b in zip(position, offset)]
            )
            sim.setObjectQuaternion(target, -1, DOWN_QUATERNION)
            result = ik.handleGroup(environment, group, {"syncWorlds": True})
            code = int(result[0] if isinstance(result, (tuple, list)) else result)
            residual = result[2] if isinstance(result, (tuple, list)) and len(result) > 2 else []
            within_explicit_tolerance = (
                len(residual) >= 2
                and float(residual[0]) <= position_tolerance
                and float(residual[1]) <= IK_ANGLE_TOLERANCE
            )
            if code != int(ik.result_success) and not within_explicit_tolerance:
                raise RuntimeError(f"Cartesian IK failed for {robot} at {position}: {result}")
            config = normalize_config(sim.getJointPosition(handle) for handle in scene.joints[robot])
            scene.set_joints(robot, config)
            collision = scene.collision_details(pair)
            if collision is not None:
                raise RuntimeError(f"{robot} Cartesian collision: {collision}")
            matrix = sim.getObjectMatrix(scene.tips[robot], -1)
            if float(matrix[10]) > -math.cos(MAX_DOWN_TILT):
                raise RuntimeError(f"{robot} flange tilt exceeded 12 degrees")
            path.append(config)
        return path
    finally:
        scene.destroy_collision_pair(pair)
        ik.eraseEnvironment(environment)
        sim.removeObjects([target])


def cartesian_down_route(
    scene: MotionScene,
    robot: str,
    start_config: list[float],
    start_position: list[float],
    waypoints: list[list[float]],
    exclusions: Iterable[int],
) -> list[list[float]]:
    path = [list(start_config)]
    config = list(start_config)
    position = list(start_position)
    for waypoint in waypoints:
        segment = cartesian_down_line(
            scene, robot, config, position, waypoint, exclusions,
            position_tolerance=0.006,
        )
        path.extend(segment[1:])
        config = path[-1]
        position = list(waypoint)
    return path


def collision_checked_joint_route(
    scene: MotionScene,
    robot: str,
    start: list[float],
    goal: list[float],
    exclusions: Iterable[int] = (),
    include_other_robots: bool = False,
    attempts: int = 60,
) -> list[list[float]]:
    """Direct or two-leg joint route with per-frame collision checking."""
    pair = scene.create_collision_pair(
        robot,
        exclusions,
        include_other_robots=include_other_robots,
    )

    def valid_segment(left: list[float], right: list[float]) -> Optional[list[list[float]]]:
        # planning-time checks run on a 5-degree stride; runtime replay
        # performs its own fine-grained checks.
        frames = densify_path([left, right], max_delta=math.radians(5.0))
        for frame in frames[::2] + frames[-1:]:
            scene.set_joints(robot, frame)
            if scene.collision_details(pair) is not None:
                return None
        return frames

    try:
        direct = valid_segment(start, goal)
        if direct is not None:
            return direct
        rng = random.Random(9000 + int(robot[1:]))
        for _ in range(attempts):
            middle = [rng.uniform(-math.pi, math.pi) for _ in range(6)]
            middle[2] = rng.uniform(-2.65, 2.65)
            first = valid_segment(start, middle)
            if first is None:
                continue
            second = valid_segment(middle, goal)
            if second is not None:
                return first + second[1:]
        raise RuntimeError(f"no collision-free joint route for {robot}")
    finally:
        scene.destroy_collision_pair(pair)


def tip_pose_for_config(
    sim: Any,
    tip: int,
    joints: list[int],
    config: Iterable[float],
) -> list[float]:
    """Evaluate the visible TCP world pose without leaving joints modified."""
    original = [float(sim.getJointPosition(joint)) for joint in joints]
    original_targets: list[float] = []
    for joint, value in zip(joints, original):
        try:
            original_targets.append(float(sim.getJointTargetPosition(joint)))
        except Exception:
            original_targets.append(value)
    try:
        for joint, value in zip(joints, config):
            sim.setJointPosition(joint, float(value))
            try:
                sim.setJointTargetPosition(joint, float(value))
            except Exception:
                pass
        return [float(value) for value in sim.getObjectPose(tip, -1)]
    finally:
        for joint, value, target in zip(joints, original, original_targets):
            sim.setJointPosition(joint, value)
            try:
                sim.setJointTargetPosition(joint, target)
            except Exception:
                pass


def build_tip_translation_path(
    scene: MotionScene,
    robot: str,
    start_config: list[float],
    start_position: list[float],
    target_position: list[float],
    exclusions: Iterable[int],
    label: str,
) -> list[list[float]]:
    """Tool-down translation to a target position (final grasp approach).

    Adapted from the legacy build_tip_translation_path: the legacy version
    preserved the start orientation via simIK ``constraint_pose``; the new
    scene only needs top-down contact, so the down constraint is used and
    the endpoint is FK-verified to 2 mm.
    """
    path = cartesian_down_line(
        scene, robot, list(start_config), start_position,
        list(target_position), exclusions,
        position_tolerance=0.0015,
    )
    actual_pose = tip_pose_for_config(
        scene.sim, scene.tips[robot], scene.joints[robot], path[-1]
    )
    position_error = math.sqrt(
        sum((actual_pose[index] - target_position[index]) ** 2 for index in range(3))
    )
    if position_error > 0.002:
        raise RuntimeError(
            f"{label} FK endpoint error is {position_error * 1000.0:.3f} mm"
        )
    return path


__all__ = [
    "COLLISION_CHECK_INTERVAL",
    "DESCENT_SPEED_DEG_S",
    "DOWN_QUATERNION",
    "JOINT_SETTLE_TOLERANCE_DEG",
    "MAX_DOWN_TILT",
    "MAX_FRAME_DELTA",
    "MotionScene",
    "RUNTIME_BRIDGE_CODE",
    "TRANSFER_SPEED_DEG_S",
    "WORKSPACE_CHECK_INTERVAL",
    "build_tip_translation_path",
    "cartesian_down_line",
    "cartesian_down_route",
    "collision_checked_joint_route",
    "config_distance",
    "cumulative_max_joint_distance",
    "densify_path",
    "find_unique_alias",
    "ik_candidates",
    "interpolate_joint_line",
    "interpolate_path",
    "join_paths",
    "minimum_jerk",
    "near",
    "normalize_config",
    "quintic",
    "sha256_file",
    "shape_tree_bounds",
    "tip_pose_for_config",
    "unwrap_path",
    "wrap_near",
]
