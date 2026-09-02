#!/usr/bin/env python3
"""Operation interface: import a cabinet model and decompose its process.

Step 1 of the automated line-planning system.  The user picks a folder of
per-part STL files (assembly coordinates, the SolidWorks export convention);
the system parses them, infers the assembly-order DAG and shows the
resulting process chain, optionally importing the parts into the open
CoppeliaSim scene for visual confirmation.

Usage:
    python3 test/import_cabinet_ui.py
"""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from test.decompose_assembly import decompose  # noqa: E402

import tkinter as tk  # noqa: E402
from tkinter import filedialog, messagebox, ttk  # noqa: E402

# ---- industrial HMI palette (matches app/main_app.py) --------------------
C_BG = "#1e2126"
C_PANEL = "#2a2e35"
C_ACCENT = "#3d8bff"
C_TEXT = "#e6e8ea"
C_OK = "#4caf50"
C_AMBER = "#ffb300"
C_RED = "#e53935"

ROLE_LABELS = {
    "base_feed": "基体上料",
    "flat_panel": "平板件安装",
    "thin_part": "杆/条类安装",
    "device": "器件安装",
    "fastener": "紧固件",
    "transfer": "转运",
}
PROCESS_LABELS = {
    "shell_feed": "箱体上料",
    "panel_install": "平板安装",
    "rail_install": "导轨安装",
    "device_install": "器件安装",
    "small_part_install": "小件安装",
    "screw": "锁付",
}


class ImportCabinetApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("柜体模型工艺拆解 — 自动化产线规划系统")
        self.root.configure(bg=C_BG)
        self.folder_path = Path(REPO_ROOT) / "test" / "models"
        self.result: dict | None = None

        self._build_ui()

    def _build_ui(self) -> None:
        header = tk.Frame(self.root, bg=C_PANEL)
        header.pack(fill=tk.X, padx=10, pady=(10, 5))
        tk.Label(
            header, text="柜体模型导入与工艺拆解",
            bg=C_PANEL, fg=C_TEXT, font=("Helvetica", 16, "bold"),
        ).pack(side=tk.LEFT, padx=10, pady=8)
        self.status_var = tk.StringVar(value="就绪 — 请导入模型文件夹")
        tk.Label(
            header, textvariable=self.status_var,
            bg=C_PANEL, fg=C_AMBER, font=("Helvetica", 10),
        ).pack(side=tk.RIGHT, padx=10)

        controls = tk.Frame(self.root, bg=C_BG)
        controls.pack(fill=tk.X, padx=10, pady=5)
        tk.Button(
            controls, text="① 导入模型文件夹", command=self._pick_folder,
            bg=C_ACCENT, fg="white", width=20,
        ).pack(side=tk.LEFT, padx=5)
        self.folder_var = tk.StringVar(value=str(self.folder_path))
        tk.Entry(
            controls, textvariable=self.folder_var, width=60,
        ).pack(side=tk.LEFT, padx=5)
        tk.Button(
            controls, text="② 解析并拆解工艺链", command=self._decompose,
            bg=C_OK, fg="white", width=20,
        ).pack(side=tk.LEFT, padx=5)
        tk.Button(
            controls, text="③ 应用到场景", command=self._apply_to_scene,
            bg=C_AMBER, fg="black", width=20,
        ).pack(side=tk.LEFT, padx=5)
        tk.Button(
            controls, text="导出 JSON", command=self._export,
            bg=C_PANEL, fg=C_TEXT, width=14,
        ).pack(side=tk.LEFT, padx=5)

        body = tk.Frame(self.root, bg=C_BG)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # process chain table
        left = tk.Frame(body, bg=C_PANEL)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tk.Label(
            left, text="工艺链（自动拆解结果）",
            bg=C_PANEL, fg=C_TEXT, font=("Helvetica", 12, "bold"),
        ).pack(anchor="w", padx=8, pady=5)
        columns = ("step", "part", "count", "process", "robot", "station")
        self.tree = ttk.Treeview(left, columns=columns, show="headings", height=22)
        for column, text, width in (
            ("step", "步序", 50), ("part", "零件", 260), ("count", "数量", 50),
            ("process", "工艺", 110), ("robot", "建议机械臂", 90), ("station", "建议工位", 100),
        ):
            self.tree.heading(column, text=text)
            self.tree.column(column, width=width, anchor="w")
        self.tree.pack(fill=tk.BOTH, expand=True, padx=8, pady=5)

        # analysis summary
        right = tk.Frame(body, bg=C_PANEL, width=320)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        tk.Label(
            right, text="分析摘要",
            bg=C_PANEL, fg=C_TEXT, font=("Helvetica", 12, "bold"),
        ).pack(anchor="w", padx=8, pady=5)
        self.summary_text = tk.Text(
            right, width=40, height=22, bg=C_BG, fg=C_TEXT,
            relief=tk.FLAT, state=tk.DISABLED, wrap=tk.WORD,
        )
        self.summary_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=5)

    def _set_status(self, text: str, color: str = C_AMBER) -> None:
        self.status_var.set(text)

    def _pick_folder(self) -> None:
        chosen = filedialog.askdirectory(
            initialdir=str(self.folder_path), title="选择包含零件 STL 的文件夹"
        )
        if chosen:
            self.folder_path = Path(chosen)
            self.folder_var.set(str(self.folder_path))

    def _decompose(self) -> None:
        folder = Path(self.folder_var.get())
        if not folder.is_dir():
            messagebox.showerror("错误", f"文件夹不存在：{folder}")
            return
        self._set_status("解析中…", C_AMBER)

        def work():
            try:
                result = decompose(folder)
            except Exception as exc:
                self.root.after(0, self._show_error, str(exc))
                return
            self.root.after(0, self._show_result, result)

        threading.Thread(target=work, daemon=True).start()

    def _show_error(self, message: str) -> None:
        self._set_status("解析失败", C_RED)
        messagebox.showerror("解析失败", message)

    def _show_result(self, result: dict) -> None:
        self.result = result
        for row in self.tree.get_children():
            self.tree.delete(row)
        for step in result["process_chain"]:
            self.tree.insert(
                "", "end",
                values=(
                    step["index"], step["part"], step.get("instances", 1),
                    PROCESS_LABELS.get(step["process"], step["process"]),
                    step["suggested_robot"], step["suggested_station"],
                ),
            )
        summary = [
            f"零件总数: {result['part_count']}",
            f"接触边: {len(result['contact_graph']['edges'])}",
            f"支撑约束: {len(result['constraints']['supports'])}",
            f"包容约束: {len(result['constraints']['contains'])}",
            "",
            "装配顺序:",
        ]
        summary.extend(f"  {step['index']}. {step['part']}" for step in result["process_chain"])
        self.summary_text.configure(state=tk.NORMAL)
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", "\n".join(summary))
        self.summary_text.configure(state=tk.DISABLED)
        self._set_status(
            f"拆解完成 — {result['part_count']} 个零件、{len(result['process_chain'])} 道工序",
            C_OK,
        )

    def _export(self) -> None:
        if self.result is None:
            messagebox.showwarning("提示", "请先完成解析")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialfile="process_chain.json",
            filetypes=[("JSON", "*.json")],
        )
        if path:
            Path(path).write_text(
                json.dumps(self.result, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self._set_status(f"已导出 {path}", C_OK)

    def _apply_to_scene(self) -> None:
        if self.result is None:
            messagebox.showwarning("提示", "请先完成解析")
            return

        def work():
            try:
                from coppeliasim_zmqremoteapi_client import RemoteAPIClient

                client = RemoteAPIClient("127.0.0.1", 23000)
                sim = client.require("sim")
                if int(sim.getSimulationState()) != int(sim.simulation_stopped):
                    raise RuntimeError("请先停止仿真")
                folder = Path(self.result["source_folder"])
                parent = sim.getObject("/FiveCR5A_Cell/Parts")
                imported = 0
                for step in self.result["process_chain"]:
                    path = folder / f"{step['part']}.STL"
                    if not path.is_file():
                        path = folder / f"{step['part']}.stl"
                    if not path.is_file():
                        continue
                    handle = sim.importShape(0, str(path), 0, 0, 1.0)
                    if isinstance(handle, int):
                        handle = handle
                    else:
                        handle = int(handle[0]) if handle else -1
                    if handle < 0:
                        continue
                    sim.setObjectAlias(handle, f"TEST_{step['part']}", {"aliasIndex": 0})
                    sim.setObjectParent(handle, parent, True)
                    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
                    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
                    imported += 1
                sim.saveScene(str(REPO_ROOT / "scenes" / "compact_cell.ttt"))
                message = f"已导入 {imported} 个零件到场景（TEST_ 前缀），场景已保存"
            except Exception as exc:
                message = f"场景应用失败: {exc}"
            self.root.after(0, self._set_status, message, C_OK)

        threading.Thread(target=work, daemon=True).start()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    root = tk.Tk()
    app = ImportCabinetApp(root)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
