"""Product colour and shell-group helpers shared by scheduler views.

Salvaged from the removed standalone demo entry
(``scripts/run_coppelia_order_demo.py``).  The colour table now follows the
new production line: two hollow-cube control-cabinet variants distinguished
by colour only — blue and red.
"""

CABINET_BLUE = [0.10, 0.42, 0.90]
CABINET_RED = [0.85, 0.16, 0.12]

# Legacy product-type keys (A/B/C) stay mapped for the current scheduler
# process model; they will be replaced by BLUE/RED in the next phase.
PRODUCT_COLORS = {
    "A": CABINET_BLUE,
    "B": CABINET_RED,
    "C": CABINET_BLUE,
    "BLUE": CABINET_BLUE,
    "RED": CABINET_RED,
    "URGENT": [1.00, 0.05, 0.05],
    "COMPLETED": [0.55, 0.58, 0.62],
}

PROCESS_TO_PRODUCT_SHELL_GROUPS = {
    "box_feed": ("assembly",),
    "pcb_install": ("assembly",),
    "module_install": ("assembly",),
    "terminal_install": ("assembly",),
    "transfer_to_inspection": ("assembly", "inspection"),
    "inspect": ("inspection",),
    "screw": ("inspection",),
    "sort_good": ("inspection",),
    "sort_defect": ("inspection",),
}


def product_color_for(product_type: str, priority: int) -> list[float]:
    color_key = "URGENT" if priority >= 5 else product_type
    return list(PRODUCT_COLORS.get(color_key, [1.0, 1.0, 1.0]))


def shell_groups_for_process(process: str) -> tuple[str, ...]:
    return PROCESS_TO_PRODUCT_SHELL_GROUPS.get(process, ())
