#!/usr/bin/env python3
"""Rewrite the report with the verified eight-arm algorithm results.

The source DOCX is never overwritten.  Existing paragraph/table structure,
floating drawings, equations, styles and relationships are retained wherever
possible; only evidence-related text, three tables and two result figures are
updated.
"""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "智调群臂报告.docx"
OUTPUT = ROOT / "报告" / "智调群臂报告_按八臂验证数据改写_20260911.docx"
DATA_DIR = ROOT / "data" / "quick_eight_arm_validation"
FIGURE_DIR = ROOT / "报告" / "figures_20260911"


def set_paragraph_text(paragraph, text: str) -> None:
    """Replace text while preserving drawings and the paragraph properties."""
    text_nodes = paragraph._p.xpath('.//*[local-name()="t"]')
    if text_nodes:
        text_nodes[0].text = text
        for node in text_nodes[1:]:
            node.text = ""
        return
    run = paragraph.add_run(text)
    if paragraph.runs and len(paragraph.runs) > 1:
        source_rpr = paragraph.runs[0]._r.rPr
        if source_rpr is not None:
            run._r.insert(0, deepcopy(source_rpr))


def set_cell_text(cell, text: str) -> None:
    first = cell.paragraphs[0]
    set_paragraph_text(first, text)
    for paragraph in cell.paragraphs[1:]:
        set_paragraph_text(paragraph, "")


def fill_table(table, rows: list[list[str]]) -> None:
    if len(rows) != len(table.rows) or any(len(row) != len(table.columns) for row in rows):
        raise ValueError(
            f"table shape mismatch: document={len(table.rows)}x{len(table.columns)}, "
            f"replacement={len(rows)}x{len(rows[0])}"
        )
    for row, values in zip(table.rows, rows):
        for cell, value in zip(row.cells, values):
            set_cell_text(cell, value)


def configure_plot_style() -> None:
    mpl.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK JP", "DejaVu Sans"],
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 10,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.axisbelow": True,
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linewidth": 0.5,
    })


def create_figures(summary: dict) -> tuple[Path, Path]:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    configure_plot_style()
    colors = ["#7F7F7F", "#56B4E9", "#E69F00"]
    labels = ["串行", "FIFO", "动态"]
    results = {item["policy"]: item for item in summary["example_results"]}

    fig, axes = plt.subplots(1, 2, figsize=(6.0, 2.53), constrained_layout=True)
    makespan = [results[key]["makespan"] for key in ("serial", "fifo", "dynamic")]
    tardiness = [results[key]["weighted_tardiness"] for key in ("serial", "fifo", "dynamic")]
    axes[0].bar(labels, makespan, color=colors, edgecolor="black", linewidth=0.5)
    axes[0].set_ylabel("总完工时间（轨迹帧）")
    axes[0].set_title("(a) 总完工时间")
    axes[0].set_ylim(0, max(makespan) * 1.18)
    axes[1].bar(labels, tardiness, color=colors, edgecolor="black", linewidth=0.5)
    axes[1].set_ylabel("加权延期（轨迹帧）")
    axes[1].set_title("(b) 加权延期")
    axes[1].set_ylim(0, max(tardiness) * 1.18)
    for ax, values in zip(axes, (makespan, tardiness)):
        for index, value in enumerate(values):
            ax.text(index, value, f"{value:.0f}", ha="center", va="bottom", fontsize=7)
    fig1 = FIGURE_DIR / "figure_6_1_eight_arm_kpi.png"
    fig.savefig(fig1, dpi=300, facecolor="white")
    plt.close(fig)

    deltas = summary["paired_delta_dynamic_minus_fifo"]
    metrics = [
        ("总完工", "makespan"),
        ("加权延期", "weighted_tardiness"),
        ("急单响应", "urgent_response"),
        ("急单完成", "urgent_completion"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(6.0, 2.53), constrained_layout=True)
    for ax, (label, key) in zip(axes.flat, metrics):
        item = deltas[key]
        mean = item["mean"]
        low = item["ci95_low"]
        high = item["ci95_high"]
        ax.axvline(0, color="#444444", linestyle="--", linewidth=0.8)
        ax.errorbar(
            mean,
            0,
            xerr=np.array([[mean - low], [high - mean]]),
            fmt="o",
            color="#0072B2",
            ecolor="#0072B2",
            capsize=3,
            markersize=4,
        )
        ax.set_yticks([])
        ax.set_title(label)
        ax.set_xlabel("动态−FIFO（轨迹帧，95% CI）")
        span = max(abs(low), abs(high), 1.0)
        ax.set_xlim(min(low - 0.15 * span, -0.1 * span), max(high + 0.15 * span, 0.1 * span))
    fig2 = FIGURE_DIR / "figure_6_2_paired_ci.png"
    fig.savefig(fig2, dpi=300, facecolor="white")
    plt.close(fig)
    return fig1, fig2


def replace_anchored_image(document: Document, paragraph_index: int, image_path: Path) -> None:
    paragraph = document.paragraphs[paragraph_index]
    rids = []
    for element in paragraph._p.xpath('.//*[local-name()="blip"]'):
        rid = element.get(qn("r:embed"))
        if rid:
            rids.append(rid)
    if len(rids) != 1:
        raise ValueError(f"expected one image at paragraph {paragraph_index}, found {rids}")
    image_part = document.part.related_parts[rids[0]]
    image_part._blob = image_path.read_bytes()


def enable_field_updates(document: Document) -> None:
    settings = document.settings._element
    existing = settings.find(qn("w:updateFields"))
    if existing is None:
        existing = OxmlElement("w:updateFields")
        settings.append(existing)
    existing.set(qn("w:val"), "true")


def main() -> int:
    summary = json.loads((DATA_DIR / "summary.json").read_text(encoding="utf-8"))
    cbs = json.loads((DATA_DIR / "cbs_summary.json").read_text(encoding="utf-8"))
    rrt = json.loads((DATA_DIR / "rrt_summary.json").read_text(encoding="utf-8"))
    fig1, fig2 = create_figures(summary)

    document = Document(SOURCE)
    replacements = {
        2: (
            "摘 要：针对多品种、小批量装配中工序调整频繁、急单响应与多机械臂共享区域冲突难以统一处理的问题，"
            "本文面向八台DOBOT CR5A组成的电控柜柔性装配场景，构建工序DAG、动态调度、固定轨迹管理与区域级冲突搜索相结合的验证方案。"
            "场景包含R1—R8、30个装配动作、三个公共工作区以及标准/减配电控柜配方；离散事件模型直接采用现有轨迹帧数作为工时输入。"
            "固定四柜试验表明，流水FIFO相对整柜串行将总完工时间缩短42.5%；在总完工时间不变的条件下，综合动态策略相对FIFO将加权延期降低55.5%，"
            "急单响应时间缩短82.3%，急单完成时间缩短42.3%。200组随机订单配对试验中，动态策略相对FIFO的加权延期差值均值为−6273.97帧，"
            "95%置信区间为[−7453.94，−5093.99]帧；急单完成时间差值均值为−873.70帧，95%置信区间为[−1182.23，−565.17]帧。"
            "区域级CBS在相向占用、双区域交叉、急单让行和三臂连锁四类用例中均获得无时间窗冲突解；六条代表关节路径上的60次RRT-Connect参考走廊搜索全部成功。"
            "上述结果验证了调度与区域时间窗协调算法，但RRT-Connect结果仅属于离线搜索模块验证，不能替代在线连续几何碰撞与实机安全验收。"
        ),
        25: (
            "第三，八机械臂场景当前以场景绑定的30条固定关节轨迹和确定性回放为主要执行方式。"
            "本轮已完成RRT-Connect参考走廊离线搜索和区域时间窗CBS算法验证；在线逆运动学、Pilz/CHOMP/STOMP/cuRobo多规划器、"
            "Ruckig时间参数化、连续几何约束CBS及动态局部重规划尚未完成场景级接入。"
        ),
        26: (
            "第四，为避免单次确定性结果受订单顺序影响，本文补充200组随机订单配对试验，并报告配对差值均值与95%置信区间。"
            "统计结果用于判断综合调度相对FIFO的收益是否可重复，同时将置信区间跨越零的指标解释为尚无明确差异。"
        ),
        28: (
            "本文围绕最新八机械臂电控柜装配场景开展研究：首先建立R1—R8资源分工、30个装配动作、三个公共工作区以及多订单流水工艺；"
            "其次将订单转换为细粒度工序DAG，构建综合动态评分调度；再将区域时间窗CBS与RRT-Connect参考走廊搜索拆分为可独立复现的最小验证模块；"
            "最后用固定轨迹帧工时、固定四柜、200组随机配对试验和权重消融构成本轮证据链。"
            "CoppeliaSim在线连续几何规划、Ruckig时间参数化及八臂实机安全验收作为后续阶段，不纳入本轮完成结论。"
        ),
        228: "3.6对比方案与权重消融设计",
        229: (
            "为区分并行能力与动态评分的作用，本文设置整柜串行、流水FIFO和综合动态调度三类可执行对比，并通过权重消融分析优先级、交期、等待和关键路径项的作用。"
            "表3.3同时保留关键路径基线的模型定义；由于当前八臂工艺为固定三级路线，其独立收益不作为本轮验证结论。"
        ),
        230: "表3.3 调度对比与权重消融方案",
        232: (
            "本轮先用整柜串行与流水FIFO分离并行资源带来的节拍收益，再用综合动态策略检验急单、交期与等待信息的作用。"
            "关键路径项作为消融配置保留，但在当前固定三级工艺中去除该项未改变结果，因此不单独宣称其性能收益。"
        ),
        237: (
            "调度结果需要转化为R1—R8可执行的运动轨迹。当前项目已建立30条场景绑定固定关节轨迹、路径插值、场景指纹校验、工作空间与安全间隙检查及确定性回放。"
            "本轮从R1、R2、R3、R4、R6和R8各选一条代表路径，在已验收轨迹周围0.32 rad参考走廊内执行RRT-Connect离线搜索，每条路径采用10个固定随机种子。"
            "该试验验证搜索模块的可执行性与重复性，不重新证明场景几何无碰撞。"
        ),
        253: (
            "这些方法按工艺段组合使用，而非全部串行执行。当前可执行主线仍采用受约束固定JSON轨迹；RRT-Connect已完成参考走廊离线算法验证，"
            "Pilz、CHOMP、STOMP、cuRobo及Ruckig保留为候选接入路线，在获得统一在线碰撞模型与时间参数接口前不计入已验证完成度。"
        ),
        292: (
            "调度优先级为CBS提供让行偏好。最小R7/R8用例中，无协调方案存在1个区域时间窗冲突；固定普通任务优先互斥使急单完成时刻由11延迟至23；"
            "优先级CBS通过延迟普通任务将冲突降为0，并保持急单完成时刻为11。"
        ),
        313: (
            "本轮RRT-Connect最小验证选取六条代表路径，每条运行10个固定随机种子，共60次。各路径搜索成功率均为100%，P95计算时间范围为0.16～115.16 ms。"
            "有效性判定采用现有验收轨迹周围的参考走廊，因此这些数值只说明搜索实现和随机重复性，不能等同于完整场景的在线碰撞规划性能。"
        ),
        333: (
            "本章建立了从固定路径执行到在线规划扩展的分层方案。现有主线完成APP/TCP/PARK点位、30条固定JSON轨迹、场景指纹和确定性回放；"
            "RRT-Connect完成六条代表路径共60次参考走廊搜索，区域级CBS完成四类时间窗冲突用例。"
            "连续几何约束下的在线RRT-Connect、多规划器融合、Ruckig及动态局部重规划仍待场景级验证。"
        ),
        404: (
            "系统采用相同八臂轨迹帧数和三级公共区约束，对整柜串行、流水FIFO和综合动态调度进行回放，并进一步开展200组随机订单配对试验。"
            "验收同时检查工艺前驱、模块容量和公共区约束，不只比较最终KPI。"
        ),
        409: (
            "本轮直接以30条固定JSON轨迹的帧数作为各模块工时输入，使调度代价与现有运动资产建立最小耦合。"
            "该方法能够反映不同机器人和工艺段的相对时间差异，但尚未包含在线规划耗时、连续几何最小间隙和控制器实测时间。"
        ),
        414: (
            "验证构造相向占用、双区域交叉、急单让行和三臂连锁四类用例。四类用例均在100个CBS节点预算内获得无时间窗冲突解，最大展开22个节点。"
            "该结果支持区域时间窗层的冲突消解逻辑，但不替代连续臂间距离检查。"
        ),
        431: (
            "灵敏度分析在同一200组随机订单集上分别去除优先级、交期、等待和关键路径项，并对优先级与交期权重施加0.5倍和1.5倍扰动。"
            "所有配置采用相同订单和轨迹工时，比较平均总完工时间、加权延期和急单完成时间。"
        ),
        436: (
            "消融结果显示，去除优先级项后平均加权延期由32631.2帧增加至34866.1帧；将优先级权重提高至1.5倍后降至31798.2帧。"
            "等待项影响较小，去除关键路径项未改变当前结果，说明固定三级工艺路线对关键路径权重不敏感。"
            "因此本文不将当前权重解释为全局最优，也不宣称关键路径项已有独立性能收益。"
        ),
        478: (
            "测试体系包括配置与数据单测、调度离散事件测试、区域时间窗CBS、RRT-Connect参考走廊搜索及CoppeliaSim/实物分层验证。"
            "本轮新增四项最小算法回归后，共运行96项自动测试并全部通过，另有1项旧五臂产品流程测试因八臂重构保持跳过。"
        ),
        486: (
            "验证目标包括三个层次：第一，确认八臂30条轨迹、三级公共区和多订单流水能够形成可计算的调度模型；第二，比较串行、FIFO与综合动态策略，"
            "并通过随机重复和权重消融识别收益边界；第三，验证区域时间窗CBS和RRT-Connect最小搜索模块。故障恢复、连续几何规划和实机安全不纳入本轮完成结论。"
        ),
        488: (
            "调度实验直接读取当前八臂轨迹库的30条动作帧数，按R1—R8固定分工、三个公共工作区和标准/减配配方构建三级离散事件模型。"
            "固定试验包含四个电控柜，其中一个为后到急单；随机试验采用固定随机种子生成200组四柜订单，随机改变到达时刻、优先级、交期和配方。"
            "同一随机场景分别运行FIFO与综合动态策略，并对KPI差值计算均值和95%置信区间。"
        ),
        489: (
            "主要指标为总完工时间、加权延期、急单响应时间和急单完成时间。统计结果采用“动态策略减FIFO”的配对差值，负值表示动态策略更优；"
            "95%置信区间跨越零时，只报告两方案尚无明确差异，不使用“显著提高”等表述。时间单位为轨迹帧，仅用于同一轨迹资产下的相对比较。"
        ),
        496: "6.3 八机械臂固定四柜调度与权重分析",
        497: "图6.1 八机械臂固定四柜三种调度策略的主要KPI",
        498: "表6.1 八机械臂固定四柜调度结果",
        500: (
            "固定四柜样例中，整柜串行、流水FIFO和综合动态策略的总完工时间分别为34202、19652和19652帧。"
            "FIFO相对串行缩短42.5%，说明三级公共区流水并行能够直接降低总完工时间。"
        ),
        501: (
            "综合动态策略与FIFO具有相同总完工时间，但加权延期由56923.2降至25315.0帧，降低55.5%；急单响应由6946.3降至1226.3帧，缩短82.3%；"
            "急单完成由18018.3降至10398.3帧，缩短42.3%。这表明当前收益主要表现为订单服务水平改善，而非进一步压缩总完工时间。"
        ),
        502: "6.4 随机统计、CBS与RRT-Connect最小验证",
        503: "图6.2 200组随机订单中动态策略相对FIFO的KPI配对差值及95%置信区间",
        504: "表6.2 200组随机订单配对统计与权重消融结果",
        506: (
            "200组随机订单中，动态策略相对FIFO的加权延期差值均值为−6273.97帧，95%置信区间为[−7453.94，−5093.99]；"
            "急单完成时间差值均值为−873.70帧，95%置信区间为[−1182.23，−565.17]。总完工时间差值区间[−118.93，19.09]跨越零，"
            "因此不认为动态策略对总完工时间存在稳定优势。区域级CBS四类用例均获得无时间窗冲突解；RRT-Connect六条代表路径共60次参考走廊搜索全部成功。"
        ),
        508: (
            "一体化GUI、统一编排器和CoppeliaSim执行器已经覆盖八臂任务状态、公共区互斥、场景指纹和30个装配动作。"
            "本轮进一步将30条轨迹帧数接入离散事件验证，完成200组随机调度、权重消融、四类区域CBS及六条代表路径的RRT-Connect离线搜索。"
            "软件算法验证与在线几何运动验收仍分层说明。"
        ),
        510: "表6.3 本轮分层验证完成度与能力边界",
        513: (
            "方案针对换型、急单、共享设备等待和多机械臂通行冲突。当前证据表明，配置驱动的三级流水和综合动态评分能够改善八臂场景中的完工时间、加权延期与急单响应，"
            "区域时间窗CBS能够处理预设冲突用例。在线几何规划落地后，新点位和布局变化才可由场景更新自动触发规划。"
        ),
        520: (
            "当前局限包括：调度时间采用轨迹帧而非控制器实测秒值；随机试验限定为四柜、标准/减配两种配方；CBS只验证区域时间窗，不包含连续几何距离；"
            "RRT-Connect只在已验收轨迹周围参考走廊内离线搜索。Pilz、CHOMP、STOMP、cuRobo、Ruckig、动态局部重规划和八臂实机整线尚未形成验证闭环。"
        ),
        521: (
            "后续优先恢复CoppeliaSim在线接口，对六条代表路径执行统一碰撞模型下的多随机种子规划，并报告成功率、规划时间、最小间隙和终点误差；"
            "随后将区域CBS与连续臂间距离检查耦合，再接入Ruckig和局部重规划。实机阶段按单臂空载、双臂共享区和多臂流水逐级推进。"
        ),
        523: (
            "迁移路线沿同一八机械臂任务模型推进。当前离散事件层已经验证多订单调度、权重敏感性和区域时间窗CBS；CoppeliaSim层继续承担完整工序、"
            "连续几何碰撞和固定轨迹回放；具备条件后再按单臂、双臂共享区和多臂流水顺序接入真实设备。"
        ),
        528: (
            "本章基于当前八机械臂30条轨迹完成固定四柜、200组随机订单、权重消融、四类区域CBS和60次RRT-Connect参考走廊搜索。"
            "结果表明，流水FIFO相对串行缩短总完工时间42.5%；综合动态策略在不增加总完工时间的情况下，将固定样例加权延期降低55.5%，并改善急单响应。"
            "随机试验进一步支持加权延期和急单完成时间的可重复改善。上述结论限于调度与区域时间窗层；在线连续几何规划、Ruckig和实机安全仍待验证。"
        ),
    }
    for index, text in replacements.items():
        set_paragraph_text(document.paragraphs[index], text)

    fill_table(document.tables[4], [
        ["方案", "并行能力", "排序依据", "区域约束", "本轮用途"],
        ["A 整柜串行", "无", "订单固定顺序", "不启用流水", "最保守基线"],
        ["B 流水FIFO", "三级并行", "先到先服务", "公共区容量1", "验证流水并行价值"],
        ["C 权重消融", "三级并行", "分别去除评分项", "与B相同", "分析各权重作用"],
        ["D 综合动态", "三级并行", "优先级、交期、等待、关键路径", "公共区约束+滚动决策", "验证急单与延期权衡"],
    ])
    fill_table(document.tables[6], [
        ["能力", "当前状态", "证据边界"],
        ["八臂场景与30条固定轨迹", "已验证", "现有场景契约、轨迹资产及回归测试"],
        ["路径插值、公共区互斥与确定性回放", "已验证", "固定轨迹执行主线"],
        ["调度统计与路径帧工时耦合", "本轮已验证", "固定四柜、200组随机订单、权重消融"],
        ["RRT-Connect候选搜索", "离线算法已验证", "六条参考走廊、60次搜索；未重做在线几何碰撞"],
        ["区域级CBS", "时间窗层已验证", "四类用例无时间窗冲突；连续几何待接入"],
        ["Pilz/CHOMP/STOMP/cuRobo/Ruckig", "待验证", "当前仅作为候选技术路线"],
        ["物理机械臂连接与安全认证", "局部双臂验证", "八臂实机整线尚未接入"],
    ])
    fill_table(document.tables[8], [
        ["策略", "总完工/帧", "急单响应/帧", "急单完成/帧", "加权延期/帧"],
        ["整柜串行", "34202.0", "23492.3", "32568.3", "148367.2"],
        ["流水FIFO", "19652.0", "6946.3", "18018.3", "56923.2"],
        ["综合动态", "19652.0", "1226.3", "10398.3", "25315.0"],
        ["动态相对FIFO", "0.0%", "−82.3%", "−42.3%", "−55.5%"],
    ])
    fill_table(document.tables[9], [
        ["指标/试验", "动态−FIFO均值", "95%置信区间", "解释"],
        ["总完工时间", "−49.92", "[−118.93，19.09]", "区间跨零，无明确差异"],
        ["加权延期", "−6273.97", "[−7453.94，−5093.99]", "动态策略改善"],
        ["急单响应", "−698.32", "[−940.47，−456.16]", "动态策略改善"],
        ["急单完成", "−873.70", "[−1182.23，−565.17]", "动态策略改善"],
        ["去除优先级项", "+2234.9延期", "—", "性能下降"],
        ["优先级权重×1.5", "−833.0延期", "—", "当前样本更优"],
        ["去除关键路径项", "0.0", "—", "固定路线不敏感"],
    ])
    fill_table(document.tables[10], [
        ["验证层级", "验证内容", "本轮结果"],
        ["自动测试", "任务、轨迹、场景契约、调度、CBS、RRT最小模块", "96项通过，1项旧五臂流程跳过"],
        ["八臂调度", "固定四柜与200组随机订单", "统计结果可重复，指标定义一致"],
        ["权重分析", "4项消融、4项权重扰动", "优先级敏感；关键路径在固定路线下不敏感"],
        ["区域CBS", "四类时间窗冲突、R7/R8三策略", "均获得无时间窗冲突解"],
        ["RRT-Connect", "六条代表路径×10种子", "参考走廊搜索60/60成功"],
        ["在线几何与实机", "连续碰撞、Ruckig、实机安全", "本轮未完成，不计入已验证结论"],
    ])

    replace_anchored_image(document, 496, fig1)
    replace_anchored_image(document, 502, fig2)
    enable_field_updates(document)
    document.core_properties.title = "面向柔性装配的八机械臂自主调度与协同规划技术（按验证数据改写）"
    document.core_properties.comments = (
        "2026-09-11：依据八臂30条轨迹、200组随机调度、权重消融、区域CBS和RRT最小验证改写。"
    )
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document.save(OUTPUT)
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
