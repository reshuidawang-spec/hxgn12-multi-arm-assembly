#!/usr/bin/env python3
"""HXGN-12 电控柜八臂协同装配 — 演示界面本地服务。

流程:导入电控柜模型 → 拆解工艺 → 分配任务 → 一键运行装配任务。

所有展示数据来自项目真实配置(configs/*.yaml、models/cabinet/processed/manifest.json);
"运行装配任务"会自动复用或唤起 CoppeliaSim,等待场景就绪后真实运行
scripts/run_8arm_pipeline_assembly.py 流水线控制器,日志原样转发到界面。

用法:
    python3 scripts/assembly_demo_ui/server.py [--host 127.0.0.1] [--port 8765]
然后浏览器打开 http://127.0.0.1:8765
"""

from __future__ import annotations

import argparse
import io
import json
import os
import subprocess
import sys
import threading
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402


LAUNCHER = REPO_ROOT / "scripts" / "start_coppelia_ubuntu.sh"
CONTROLLER = REPO_ROOT / "scripts" / "run_8arm_pipeline_assembly.py"
PIPELINE_PLAN = REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.partial.json"
MODELS_ROOT = REPO_ROOT / "models"
DEFAULT_MODEL = "cabinet"
ASSIGNMENT_PATH = REPO_ROOT / "configs" / "assembly_task_assignment.yaml"
ROBOTS_PATH = REPO_ROOT / "configs" / "robots.yaml"

# 柜型号显示名:新增型号目录后在此补充中文名,未登记的按目录名显示。
MODEL_DISPLAY = {
    "cabinet": "HXGN-12 标准型",
}

SIM_HOST = "127.0.0.1"
SIM_PORT = 23000
INDEX_HTML = Path(__file__).resolve().parent / "index.html"

PART_DISPLAY = {
    "shell": "柜体(壳体)",
    "mounting_panel": "内安装板",
    "rail_h1": "水平导轨",
    "rail_v1": "竖直导轨 A",
    "rail_v2": "竖直导轨 B",
    "psu": "开关电源 LRS-350-24",
    "servo": "伺服驱动器 RYH201F5-VV2",
    "eds": "导轨交换机 EDS-205",
    "plc": "PLC 可编程器 AFPX-C60T",
    "dma": "步进驱动器 DMA882S",
    "contactor": "交流接触器 CJX2-0901",
    "breaker": "漏电保护器 DZ47LE-32",
    "com5": "通讯接插件 AFPX-COM5",
    "filter": "电源滤波器 DL-10T1",
}

TOOL_LABELS = {
    "gripper": "气动夹具",
    "magnetic_gripper": "磁吸夹爪",
    "slim_gripper": "窄爪夹具",
    "vacuum": "真空吸盘",
    "screwdriver": "电动螺丝刀",
}

ROBOT_DISPLAY = {
    "R1": "壳体上料与托盘定位",
    "R2": "水平导轨磁吸安装",
    "R3": "双竖直导轨安装",
    "R4": "电源与驱动器件安装",
    "R5": "控制器件真空安装",
    "R6": "开关保护与通讯模块",
    "R7": "四点内部紧固",
    "R8": "滤波器安装",
}

OPERATION_DISPLAY = {
    "shell_load_and_locate": "抓取柜体并定位到索引托盘",
    "horizontal_rail_install": "磁吸抓取并安装水平导轨",
    "vertical_rail_install": "依次抓取并安装两根竖直导轨",
    "power_and_drive_install": "侧夹安装电源与驱动器件",
    "control_and_filter_install": "真空吸取并安装控制器件",
    "switching_and_protection_install": "精确夹取安装开关与保护器件",
    "communication_module_install": "安装通讯接插件",
    "filter_install": "真空吸取并安装滤波器",
    "four_point_fastening": "拧紧四个内部安装紧固件",
}


def load_yaml(path: Path) -> dict:
    try:
        from scheduler.config_loader import load_yaml as _load
    except Exception:
        import yaml

        def _load(p: Path):
            with p.open(encoding="utf-8") as stream:
                return yaml.safe_load(stream)

    return _load(path)


# ---------------------------------------------------------------- shared state

_LOG_LOCK = threading.Lock()
_LOG_LINES: list[str] = []

_STATE_LOCK = threading.RLock()
_sim_proc: subprocess.Popen | None = None
_ctrl_proc: subprocess.Popen | None = None
_run_thread: threading.Thread | None = None
_sim_state = "idle"  # idle | launching | ready | running | error
_ctrl_state = "idle"  # idle | waiting | running | exited | error
_ctrl_exit_code: int | None = None


def append_log(tag: str, line: str) -> None:
    for chunk in line.splitlines() or [""]:
        with _LOG_LOCK:
            _LOG_LINES.append(f"[{tag}] {chunk}")


def tail_log(since: int) -> tuple[list[str], int]:
    with _LOG_LOCK:
        return list(_LOG_LINES[since:]), len(_LOG_LINES)


def set_sim_state(state: str) -> None:
    global _sim_state
    with _STATE_LOCK:
        _sim_state = state


def set_ctrl_state(state: str, code: int | None = None) -> None:
    global _ctrl_state, _ctrl_exit_code
    with _STATE_LOCK:
        _ctrl_state = state
        _ctrl_exit_code = code


def state_snapshot() -> dict:
    with _STATE_LOCK:
        return {
            "sim": _sim_state,
            "ctrl": _ctrl_state,
            "ctrl_exit_code": _ctrl_exit_code,
        }


_REPORT_LOCK = threading.Lock()
_report: dict = {"plan": None, "events": [], "jobs_completed": []}


def reset_report() -> None:
    """新一场装配开始前清零报告数据。"""
    with _REPORT_LOCK:
        _report["plan"] = None
        _report["events"] = []
        _report["jobs_completed"] = []


def ingest_report_line(line: str) -> None:
    """解析流水线的计划、工序和柜体完工 JSON 事件。"""
    prefixes = ("[report] ", "[report-plan] ", "[report-job] ")
    if not line.startswith(prefixes):
        return
    try:
        payload = json.loads(line.split("] ", 1)[1])
    except (IndexError, json.JSONDecodeError):
        return
    with _REPORT_LOCK:
        if line.startswith("[report-plan] "):
            _report["plan"] = payload
        elif line.startswith("[report-job] "):
            _report["jobs_completed"].append(payload)
        else:
            _report["events"].append(payload)


def report_summary() -> dict:
    """聚合:各工艺模块完工率、各臂执行时间、完工臂数。"""
    with _REPORT_LOCK:
        plan = _report["plan"]
        events = list(_report["events"])
        completed_job_events = list(_report["jobs_completed"])
    if not plan:
        return {
            "status": "empty",
            "modules": [],
            "arms": [],
            "jobs": 0,
            "jobs_completed": 0,
            "arms_completed": 0,
            "arms_total": 8,
        }
    module_plan = plan.get("modules", {})
    arm_plan = plan.get("arms", {})
    done_module = {str(m): 0 for m in module_plan}
    done_arm = {r: 0 for r in arm_plan}
    arm_seconds = {r: 0.0 for r in arm_plan}
    seen: set[tuple] = set()
    for event in events:
        key = (int(event.get("module", 0)), int(event.get("job", 0)), event.get("key"))
        if key in seen:
            continue
        seen.add(key)
        module = str(event.get("module", 0))
        if module in done_module:
            done_module[module] += 1
        robot = event.get("robot")
        if robot:
            done_arm[robot] = done_arm.get(robot, 0) + 1
            arm_seconds[robot] = arm_seconds.get(robot, 0.0) + float(
                event.get("seconds", 0.0)
            )
    modules = []
    for module in sorted(module_plan, key=int):
        total = int(module_plan[module])
        done = min(done_module.get(module, 0), total)
        modules.append(
            {
                "id": f"M{module}",
                "name": {
                    "1": "柜体与导轨基础装配",
                    "2": "电力与控制器件安装",
                    "3": "通讯、滤波与紧固",
                }.get(module, f"装配模块 {module}"),
                "done": done,
                "total": total,
                "pct": round(done / total * 100, 1) if total else 100.0,
            }
        )
    arms = []
    completed_arms = 0
    for robot in sorted(arm_plan, key=lambda value: int(value[1:])):
        total = int(arm_plan[robot])
        done = min(done_arm.get(robot, 0), total)
        if total and done >= total:
            completed_arms += 1
        arms.append(
            {
                "id": robot,
                "name": ROBOT_DISPLAY.get(robot, robot),
                "done": done,
                "total": total,
                "pct": round(done / total * 100, 1) if total else 100.0,
                "seconds": round(arm_seconds.get(robot, 0.0), 1),
            }
        )
    completed_jobs = len(
        {
            int(event.get("job", 0))
            for event in completed_job_events
            if event.get("job")
        }
    )
    planned_jobs = int(plan.get("jobs", 0))
    all_done = (
        bool(modules)
        and all(m["done"] >= m["total"] for m in modules)
        and completed_jobs >= planned_jobs
    )
    return {
        "status": "finished" if all_done else "running",
        "modules": modules,
        "arms": arms,
        "arms_completed": completed_arms,
        "arms_total": len(arm_plan),
        "jobs": planned_jobs,
        "jobs_completed": completed_jobs,
        "total_seconds": round(sum(arm_seconds.values()), 1),
    }


def pump_stream(stream, tag: str) -> None:
    for raw in iter(stream.readline, ""):
        line = raw.rstrip("\n")
        if tag == "ctrl":
            ingest_report_line(line)
        append_log(tag, line)
    stream.close()


def _probe_sim_state(timeout: float = 8.0) -> str | None:
    """有界探测仿真服务:返回 ready/running/paused,连不上返回 None。

    RemoteAPIClient 构造握手没有超时参数,无服务时会长时间阻塞,因此:
    先用 TCP 端口预检快速排除,再做有界线程 ZMQ 探测,超时视为不可用。
    """
    import socket

    try:
        with socket.create_connection((SIM_HOST, SIM_PORT), timeout=1.5):
            pass
    except OSError:
        return None
    outcome: list[str | None] = [None]

    def probe() -> None:
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient

            client = RemoteAPIClient(SIM_HOST, SIM_PORT)
            client.timeout = 4.0
            sim = client.require("sim")
            sim_state = int(sim.getSimulationState())
            if sim_state == int(sim.simulation_stopped):
                outcome[0] = "ready"
            elif sim_state == int(sim.simulation_paused):
                outcome[0] = "paused"
            else:
                outcome[0] = "running"
        except Exception:
            outcome[0] = None

    thread = threading.Thread(target=probe, daemon=True)
    thread.start()
    thread.join(timeout=timeout)
    return outcome[0]


def _sim_alive() -> bool:
    """真实 ZMQ 探测:仿真服务当前是否可连(不以缓存状态为准)。"""
    return _probe_sim_state() is not None


def launch_sim() -> tuple[bool, str]:
    """启动 CoppeliaSim(带场景),随后由轮询线程探测 ZMQ 就绪状态。"""
    global _sim_proc, _sim_state
    with _STATE_LOCK:
        proc_alive = _sim_proc is not None and _sim_proc.poll() is None
        if proc_alive:
            return True, "CoppeliaSim 正在启动或已运行,将复用当前实例"
    probed = _probe_sim_state()
    if probed is not None:
        with _STATE_LOCK:
            _sim_state = probed
        return True, "CoppeliaSim 已就绪,将复用当前实例"
    root = os.environ.get(
        "COPPELIASIM_ROOT", "/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"
    )
    if not Path(root, "coppeliaSim.sh").is_file():
        set_sim_state("error")
        return (
            False,
            f"未找到 CoppeliaSim 启动脚本:{root}/coppeliaSim.sh,"
            "请设置 COPPELIASIM_ROOT 环境变量后重试",
        )
    if not LAUNCHER.is_file():
        set_sim_state("error")
        return False, f"启动脚本不存在:{LAUNCHER}"
    env = dict(os.environ, COPPELIASIM_ROOT=root)
    try:
        _sim_proc = subprocess.Popen(
            ["bash", str(LAUNCHER)],
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
    except OSError as exc:
        set_sim_state("error")
        return False, f"无法启动 CoppeliaSim:{exc}"
    set_sim_state("launching")
    append_log("sim", f"正在唤起 CoppeliaSim (PID {_sim_proc.pid}) ...")
    threading.Thread(
        target=pump_stream, args=(_sim_proc.stdout, "sim"), daemon=True
    ).start()
    threading.Thread(target=_sim_proc_guard, daemon=True).start()
    threading.Thread(target=_poll_sim_ready, daemon=True).start()
    return True, "正在唤起 CoppeliaSim,界面将自动探测 ZMQ 就绪状态"


def _sim_proc_guard() -> None:
    """CoppeliaSim 进程退出 → 状态如实回退,避免陈旧的就绪状态。"""
    global _sim_state
    with _STATE_LOCK:
        proc = _sim_proc
    if proc is None:
        return
    code = proc.wait()
    with _STATE_LOCK:
        if _sim_state in ("launching", "ready", "running", "paused", "idle"):
            _sim_state = "error"
    append_log("sim", f"CoppeliaSim 已退出(exit={code}),界面状态已更新")


def _poll_sim_ready() -> None:
    """每 2 秒探测一次 ZMQ(有界),直到仿真服务可用。"""
    global _sim_state
    import time

    while True:
        with _STATE_LOCK:
            state = _sim_state
            proc = _sim_proc
        if state in ("ready", "running", "error"):
            return
        if proc is not None and proc.poll() is not None:
            time.sleep(2)
            continue
        probed = _probe_sim_state(timeout=6.0)
        if probed is None:
            time.sleep(2)
            continue
        with _STATE_LOCK:
            if _sim_state == "launching":
                _sim_state = probed
        append_log(
            "sim",
            "ZMQ 已连接,仿真服务"
            + ("就绪(停止态,可开始装配)" if probed == "ready"
               else "正在运行中"),
        )
        return


def controller_command(quantity: int) -> list[str]:
    """生成界面与手动演示共用的流水线命令。"""
    command = [
        sys.executable,
        str(CONTROLLER),
        "--jobs",
        str(quantity),
        "--speed",
        "3.0",
        "--plan",
        str(PIPELINE_PLAN),
    ]
    if quantity == 4:
        command.extend(["--black-job", "2", "--reduced-job", "2"])
    return command


def start_controller(quantity: int = 1) -> tuple[bool, str]:
    """运行三级流水线控制器，stdout 原样进入界面日志。"""
    global _ctrl_proc
    if not isinstance(quantity, int) or not 1 <= quantity <= 4:
        return False, "生产数量必须是 1~4 的整数(4 台为黑色简化柜混流演示)"
    with _STATE_LOCK:
        if _ctrl_proc is not None and _ctrl_proc.poll() is None:
            return False, "装配控制器已在运行"
    if not _sim_alive():
        set_sim_state("error")
        append_log(
            "sim",
            "ZMQ 探测失败:CoppeliaSim 未在运行或已退出,请先点击「运行装配任务」",
        )
        return False, "CoppeliaSim 未在运行或已退出,请先点击「运行装配任务」"
    reset_report()
    try:
        _ctrl_proc = subprocess.Popen(
            controller_command(quantity),
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except OSError as exc:
        set_ctrl_state("error")
        return False, f"无法启动装配控制器:{exc}"
    set_ctrl_state("running")
    set_sim_state("running")
    append_log(
        "ctrl",
        f"装配控制器已启动 (PID {_ctrl_proc.pid}),生产数量:{quantity} 台",
    )
    threading.Thread(
        target=pump_stream, args=(_ctrl_proc.stdout, "ctrl"), daemon=True
    ).start()
    threading.Thread(target=_ctrl_guard, daemon=True).start()
    return True, f"装配控制器已启动,开始生产 {quantity} 台电控柜"


def _ctrl_guard() -> None:
    with _STATE_LOCK:
        proc = _ctrl_proc
    if proc is None:
        return
    code = proc.wait()
    append_log("ctrl", f"装配控制器结束 (exit={code})")
    set_ctrl_state("exited", code)
    probed = _probe_sim_state()
    if probed is not None:
        set_sim_state(probed)


def _stop_live_simulation() -> bool:
    """将手动运行或上次完成后暂停的场景恢复到停止态。"""
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient(SIM_HOST, SIM_PORT)
        client.timeout = 8.0
        sim = client.require("sim")
        if int(sim.getSimulationState()) != int(sim.simulation_stopped):
            sim.stopSimulation()
        return True
    except Exception as exc:
        append_log("sim", f"无法停止当前仿真:{exc}")
        return False


def _launch_and_start(quantity: int) -> None:
    """后台一键流程:确保 GUI 存活、场景停止,然后启动装配。"""
    import time

    ok, message = launch_sim()
    append_log("system", message)
    if not ok:
        set_ctrl_state("error")
        return

    deadline = time.monotonic() + 120.0
    requested_stop = False
    while time.monotonic() < deadline:
        probed = _probe_sim_state(timeout=6.0)
        if probed == "ready":
            ok, message = start_controller(quantity)
            append_log("system", message)
            if not ok:
                set_ctrl_state("error")
            return
        if probed in ("running", "paused") and not requested_stop:
            append_log("sim", "检测到场景非停止态,正在恢复到可重放的初始状态")
            if not _stop_live_simulation():
                set_ctrl_state("error")
                return
            requested_stop = True
        time.sleep(1.0)

    append_log("system", "等待 CoppeliaSim 场景就绪超时(120 s)")
    set_sim_state("error")
    set_ctrl_state("error")


def run_assembly(quantity: int = 4) -> tuple[bool, str]:
    """接收界面的单次操作,异步完成仿真启动与装配运行。"""
    global _run_thread
    if not isinstance(quantity, int) or not 1 <= quantity <= 4:
        return False, "生产数量必须是 1~4 的整数"
    with _STATE_LOCK:
        if _ctrl_proc is not None and _ctrl_proc.poll() is None:
            return False, "装配控制器已在运行"
        if _run_thread is not None and _run_thread.is_alive():
            return False, "装配任务正在启动,请勿重复点击"
        set_ctrl_state("waiting")
        append_log(
            "system",
            f"已接收一键装配任务:生产 {quantity} 台"
            + ("(第 2 台为黑色简化工艺柜)" if quantity == 4 else ""),
        )
        _run_thread = threading.Thread(
            target=_launch_and_start,
            args=(quantity,),
            daemon=True,
            name="assembly-one-click",
        )
        _run_thread.start()
    return True, f"装配任务已接收,将自动生产 {quantity} 台电控柜"


def stop_controller() -> tuple[bool, str]:
    global _ctrl_proc
    with _STATE_LOCK:
        proc = _ctrl_proc
        if proc is None or proc.poll() is not None:
            return False, "装配控制器当前未运行"
    proc.terminate()
    append_log("ctrl", "已发送停止信号")
    return True, "已请求停止装配控制器"


# ---------------------------------------------------------------- data helpers

def available_models() -> list[dict]:
    """扫描 models/ 下所有含 processed/manifest.json 的柜型号(含概览)。"""
    entries = []
    for model_dir in sorted(MODELS_ROOT.iterdir()):
        manifest_path = model_dir / "processed" / "manifest.json"
        if not model_dir.is_dir() or not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        parts = manifest["parts"]
        total_tris = sum(info["tris_out"] for info in parts.values())
        lo = [1e9, 1e9, 1e9]
        hi = [-1e9, -1e9, -1e9]
        for info in parts.values():
            for axis in range(3):
                lo[axis] = min(lo[axis], info["bbox_lo"][axis])
                hi[axis] = max(hi[axis], info["bbox_hi"][axis])
        entries.append(
            {
                "id": model_dir.name,
                "name": MODEL_DISPLAY.get(model_dir.name, model_dir.name),
                "path": str(model_dir / "processed"),
                "count": len(parts),
                "total_tris": total_tris,
                "size_mm": [round((hi[i] - lo[i]) * 1000, 1) for i in range(3)],
            }
        )
    return entries


def _model_manifest_path(model_id: str) -> Path:
    known = {entry["id"] for entry in available_models()}
    if model_id not in known:
        raise ValueError(f"未知柜型号:{model_id}")
    return MODELS_ROOT / model_id / "processed" / "manifest.json"


def model_triangles(model_id: str, part_id: str) -> np.ndarray:
    """按型号读取已处理 STL 三角网格(缓存)。"""
    data = (_model_manifest_path(model_id).parent / f"{part_id}.stl").read_bytes()
    dtype = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data[84:], dtype=dtype)["v"].astype(float)


def model_info(model_id: str = DEFAULT_MODEL) -> dict:
    manifest = json.loads(_model_manifest_path(model_id).read_text(encoding="utf-8"))
    parts = []
    total_tris = 0
    lo = [1e9, 1e9, 1e9]
    hi = [-1e9, -1e9, -1e9]
    for key, info in sorted(manifest["parts"].items()):
        bbox_lo = info["bbox_lo"]
        bbox_hi = info["bbox_hi"]
        for axis in range(3):
            lo[axis] = min(lo[axis], bbox_lo[axis])
            hi[axis] = max(hi[axis], bbox_hi[axis])
        total_tris += info["tris_out"]
        parts.append(
            {
                "key": key,
                "display": PART_DISPLAY.get(key, key),
                "source": info["source"],
                "tris": info["tris_out"],
                "bbox_mm": [
                    round((bbox_hi[i] - bbox_lo[i]) * 1000, 1) for i in range(3)
                ],
            }
        )
    size_mm = [round((hi[i] - lo[i]) * 1000, 1) for i in range(3)]
    return {
        "id": model_id,
        "name": MODEL_DISPLAY.get(model_id, model_id),
        "parts": parts,
        "count": len(parts),
        "total_tris": total_tris,
        "size_mm": size_mm,
        "frame": manifest.get("frame", {}),
    }


@lru_cache(maxsize=1)
def assignment_info() -> dict:
    assignment = load_yaml(ASSIGNMENT_PATH)
    robots = load_yaml(ROBOTS_PATH)["robots"]
    tasks = []
    for task in assignment["tasks"]:
        tasks.append(
            {
                "id": task["id"],
                "robot": task["robot"],
                "workspace": task["workspace"],
                "operation": OPERATION_DISPLAY.get(task["id"], task["operation"]),
                "actions": task["actions"],
                "parts": task["parts"],
                "parts_display": [PART_DISPLAY.get(p, p) for p in task["parts"]],
                "tool": task["required_tool"],
                "tool_label": TOOL_LABELS.get(task["required_tool"], task["required_tool"]),
                "after": task["after"],
            }
        )
    arm_summary = {}
    for robot_id in sorted(robots, key=lambda r: int(r[1:])):
        robot = robots[robot_id]
        arm_tasks = [t for t in tasks if t["robot"] == robot_id]
        arm_summary[robot_id] = {
            "name": robot["name"],
            "name_cn": ROBOT_DISPLAY.get(robot_id, robot_id),
            "position": robot["position"],
            "model": robot["model"],
            "tasks": arm_tasks,
            "parts": [p for t in arm_tasks for p in t["parts"]],
            "parts_display": [PART_DISPLAY.get(p, p) for t in arm_tasks for p in t["parts"]],
            "tools": sorted({t["tool_label"] for t in arm_tasks}),
        }
    return {
        "schema_version": assignment["schema_version"],
        "policy": assignment["policy"],
        "workspaces": assignment["workspaces"],
        "tasks": tasks,
        "arms": arm_summary,
        "arm_order": sorted(robots, key=lambda r: int(r[1:])),
    }


def disassembly_info() -> dict:
    info = assignment_info()
    # 装配任务已按依赖拓扑排序 → 拆解即严格逆序。
    steps = []
    for seq, task in enumerate(reversed(info["tasks"]), start=1):
        parts = [PART_DISPLAY.get(p, p) for p in task["parts"]]
        steps.append(
            {
                "seq": seq,
                "task_id": task["id"],
                "robot": task["robot"],
                "operation": task["operation"],
                "parts": parts,
                "tool": task["tool_label"],
            }
        )
    return {
        "steps": steps,
        "count": len(steps),
        "notes": [
            f"预装件「{PART_DISPLAY.get(p, p)}」不参与拆解/装配工序"
            for p in info["policy"].get("preassembled_parts", [])
        ],
    }


_PREVIEW_LOCK = threading.Lock()
_PREVIEW_CACHE: dict[str, bytes] = {}

# SolidWorks 默认蓝灰材质基调(成品柜截图风格),法线光照负责立体感。
SW_TONES = {
    "shell": (202, 210, 221),
    "default": (196, 205, 217),
}
HIGHLIGHT_COLOR = (14, 165, 233)  # 深青,浅色背景上依然醒目
_LIGHT_DIR = np.array([0.48, -0.42, 0.95])
_LIGHT_DIR = _LIGHT_DIR / np.linalg.norm(_LIGHT_DIR)
_FILL_DIR = np.array([-0.62, 0.15, 0.42])
_FILL_DIR = _FILL_DIR / np.linalg.norm(_FILL_DIR)
_VIEW_DIR = np.array(
    [
        np.cos(np.radians(30.0)) * np.sin(np.radians(-50.0)),
        np.cos(np.radians(30.0)) * np.cos(np.radians(-50.0)),
        np.sin(np.radians(30.0)),
    ]
)  # 与预览视角一致,用于高光计算


def _project(mesh, elev: float, azim: float):
    """SolidWorks 风格透视投影 → 屏幕坐标 (N,3,2) 与深度 (N,3)。

    先做 view_init 等价旋转,再按视线距离做透视缩放(f/d),z2 越大越靠近
    观察者,供画家算法排序。
    """
    import math

    az = math.radians(azim)
    el = math.radians(elev)
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)
    x, y, z = mesh[..., 0], mesh[..., 1], mesh[..., 2]
    x1 = ca * x - sa * y
    y1 = sa * x + ca * y
    x2 = x1
    y2 = ce * y1 - se * z
    z2 = se * y1 + ce * z  # 越大越靠近观察者
    camera_distance = 4.5
    focal = 4.5
    scale = focal / (camera_distance - z2)
    return np.stack([x2 * scale, y2 * scale], axis=-1), z2


def _face_normals(mesh) -> np.ndarray:
    v0, v1, v2 = mesh[:, 0], mesh[:, 1], mesh[:, 2]
    normals = np.cross(v1 - v0, v2 - v0)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.divide(
        normals, length, out=np.zeros_like(normals), where=length > 1e-12
    )


def _sw_shades(mesh, base_rgb: tuple) -> np.ndarray:
    """SolidWorks 风格光照:环境光 + 主/辅光漫反射 + Blinn 高光,返回 0..1 RGB。"""
    normals = _face_normals(mesh)
    diffuse = np.clip(normals @ _LIGHT_DIR, 0.0, 1.0)
    fill = np.clip(normals @ _FILL_DIR, 0.0, 1.0)
    half = _LIGHT_DIR + _VIEW_DIR
    half = half / np.linalg.norm(half)
    specular = np.power(np.clip(normals @ half, 0.0, 1.0), 32.0) * 0.55
    intensity = 0.48 + 0.50 * diffuse + 0.16 * fill + specular
    base = np.array(base_rgb, dtype=float) / 255.0
    return np.clip(base[None, :] * intensity[:, None], 0.0, 1.0)


def preview_png(highlight: str | None = None, model_id: str = DEFAULT_MODEL) -> bytes:
    """成品柜预览(PNG,内存缓存),SolidWorks 截图风格。

    浅灰渐变背景 + 地面软阴影 + 透视投影 + 蓝灰材质双光源/高光;
    highlight 指定的零件以深青色自发光点亮,用于零件清单点击高亮。
    不依赖 mplot3d:手工做旋转/透视投影,全柜三角形按深度全局排序后
    用 PolyCollection 画家算法绘制。
    """
    cache_key = f"{model_id}:{highlight or ''}"
    with _PREVIEW_LOCK:
        cached = _PREVIEW_CACHE.get(cache_key)
        if cached is not None:
            return cached
        caps = {"shell": 9000}
        patches = []
        colors = []
        depths = []
        for entry in model_info(model_id)["parts"]:
            key = entry["key"]
            mesh = model_triangles(model_id, key)
            cap = caps.get(key, 2500)
            if len(mesh) > cap:
                step = max(1, len(mesh) // cap)
                mesh = mesh[::step]
            screen, depth = _project(mesh, elev=30.0, azim=-50.0)
            tri_depth = depth.mean(axis=1)
            if key == highlight:
                base = np.array(HIGHLIGHT_COLOR, dtype=float) / 255.0
                shades = np.tile(base, (len(mesh), 1))
            else:
                shades = _sw_shades(
                    mesh, SW_TONES.get(key, SW_TONES["default"])
                )
            patches.extend(tri for tri in screen)
            colors.extend(shades)
            depths.append(tri_depth)
        order = np.argsort(np.concatenate(depths))  # 远 → 近,画家算法
        ordered = [patches[i] for i in order]
        ordered_colors = [colors[i] for i in order]
        fig, ax = plt.subplots(figsize=(9, 6.2), facecolor="#eef1f5")
        ax.set_facecolor("#eef1f5")
        shell_screen, _ = _project(model_triangles(model_id, "shell"), 30.0, -50.0)
        all_xy = shell_screen.reshape(-1, 2)
        pad = 0.10
        x0, x1 = all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad
        y0, y1 = all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        # 浅灰垂直渐变背景(顶部亮、底部灰),SolidWorks 画布观感
        grad = np.linspace(
            np.array([246, 248, 251]) / 255.0,
            np.array([206, 213, 222]) / 255.0,
            256,
        )[:, None, :]
        ax.imshow(
            grad,
            extent=[x0, x1, y0, y1],
            aspect="auto",
            origin="lower",
            zorder=0,
            interpolation="bilinear",
        )
        # 地面软阴影:柜底投影位置画两层半透明椭圆
        theta = np.linspace(0.0, 2.0 * np.pi, 64)
        for rx, ry, alpha in ((0.24, 0.135, 0.12), (0.30, 0.17, 0.06)):
            ellipse = np.stack(
                [rx * np.cos(theta), ry * np.sin(theta), np.zeros(64)], axis=-1
            )
            shadow_xy, _ = _project(ellipse, 30.0, -50.0)
            ax.fill(
                shadow_xy[:, 0],
                shadow_xy[:, 1],
                color=(0.02, 0.05, 0.1),
                alpha=alpha,
                zorder=1,
                linewidth=0,
            )
        ax.add_collection(
            PolyCollection(
                ordered, facecolors=ordered_colors, edgecolors="none", alpha=1.0
            )
        )
        ax.set_aspect("equal")
        ax.axis("off")
        fig.tight_layout(pad=0.25)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=130, facecolor=fig.get_facecolor())
        plt.close(fig)
        _PREVIEW_CACHE[cache_key] = buf.getvalue()
        return _PREVIEW_CACHE[cache_key]


# ---------------------------------------------------------------- HTTP handler

class Handler(BaseHTTPRequestHandler):
    server_version = "AssemblyDemoUI/1.0"

    def log_message(self, fmt, *args):  # 静默访问日志
        pass

    def _json(self, payload: dict, code: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _ok(self, payload: dict | None = None, message: str = "ok") -> None:
        self._json({"ok": True, "message": message, **(payload or {})})

    def _fail(self, message: str, code: int = 400) -> None:
        self._json({"ok": False, "message": message}, code)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                body = INDEX_HTML.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/state":
                self._ok({"state": state_snapshot()})
            elif path == "/api/log":
                since = int(parse_qs(urlparse(self.path).query).get("since", ["0"])[0])
                lines, total = tail_log(since)
                self._ok({"lines": lines, "total": total})
            elif path == "/api/sim_status":
                self._ok({"state": state_snapshot()})
            elif path == "/api/models":
                self._ok({"models": available_models()}, "柜型号列表")
            elif path == "/api/report":
                self._ok({"report": report_summary()}, "完工报告")
            elif path == "/preview.png":
                query = parse_qs(urlparse(self.path).query)
                part = query.get("part", [None])[0]
                model = query.get("model", [DEFAULT_MODEL])[0]
                try:
                    png = preview_png(part, model)
                except ValueError as exc:
                    self._fail(str(exc), 404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.end_headers()
                self.wfile.write(png)
            else:
                self._fail("not found", 404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self._fail(f"内部错误:{exc}", 500)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/import_model":
                body = self._read_body()
                model_id = str(body.get("model") or DEFAULT_MODEL)
                try:
                    info = model_info(model_id)
                except ValueError as exc:
                    self._fail(str(exc))
                    return
                self._ok(info, f"{info['name']} 模型导入成功")
            elif path == "/api/disassemble":
                self._ok(disassembly_info(), "拆解工艺生成成功")
            elif path == "/api/assign":
                self._ok(assignment_info(), "任务已成功分配到各个机械臂")
            elif path == "/api/launch_sim":
                ok, message = launch_sim()
                self._ok({"sim_state": state_snapshot()["sim"]}, message) if ok else self._fail(message)
            elif path == "/api/start_assembly":
                body = self._read_body()
                try:
                    quantity = int(body.get("quantity", 1))
                except (TypeError, ValueError):
                    quantity = -1
                ok, message = start_controller(quantity)
                self._ok({"ctrl_state": state_snapshot()["ctrl"]}, message) if ok else self._fail(message)
            elif path == "/api/run_assembly":
                body = self._read_body()
                try:
                    quantity = int(body.get("quantity", 4))
                except (TypeError, ValueError):
                    quantity = -1
                ok, message = run_assembly(quantity)
                self._ok({"state": state_snapshot()}, message) if ok else self._fail(message)
            elif path == "/api/stop_assembly":
                ok, message = stop_controller()
                self._ok(message=message) if ok else self._fail(message)
            else:
                self._fail("not found", 404)
        except BrokenPipeError:
            pass
        except Exception as exc:
            self._fail(f"内部错误:{exc}", 500)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}"
    print(f"装配演示界面已启动:{url}")
    print("按 Ctrl+C 退出(界面服务不影响已启动的 CoppeliaSim/装配控制器)")

    def discover_existing_sim() -> None:
        probed = _probe_sim_state()
        if probed is not None:
            set_sim_state(probed)
            append_log("sim", f"已发现现有 CoppeliaSim 实例,状态:{probed}")

    threading.Thread(
        target=discover_existing_sim,
        daemon=True,
        name="initial-sim-discovery",
    ).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n界面服务已退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
