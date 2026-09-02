#!/usr/bin/env python3
"""Plan and run the eight-arm control-cabinet assembly in CoppeliaSim.

The controller deliberately uses the target dummies built into compact_cell.ttt.
It first converts every APP/TCP pair to a fixed joint-space path with simIK,
then performs a robot-robot collision preflight, and finally replays the paths
with deterministic stepping.  Parts are attached to the tool at pick TCPs and
snapped to the assembled-product reference geometry at place TCPs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from sim_bridge.scene_objects import ARM_JOINT_ALIASES, POINTS, ROBOT_IDS, ROBOT_TIPS


SCENE_ROOT = "/FiveCR5A_Cell"
SCENE_FILE = REPO_ROOT / "scenes" / "compact_cell.ttt"
DEFAULT_PLAN = REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.json"
DOWN_QUATERNION = [1.0, 0.0, 0.0, 0.0]  # CoppeliaSim order: x, y, z, w
HOME = [0.0] * 6
MAX_FRAME_DELTA = math.radians(2.0)
QUINTIC_MAX_SLOPE = 1.875
MAX_DOWN_TILT = math.radians(15.0)
MAX_TRANSFER_TILT = math.radians(40.0)
IK_POSITION_TOLERANCE = 0.003
IK_ANGLE_TOLERANCE = math.radians(2.0)
TRANSIT_POSITION_TOLERANCE = 0.015
PLAN_SCHEMA_VERSION = 31
MIN_TRANSIT_Z = 0.44
PREFERRED_TRANSIT_Z = 0.50
TRANSIT_CLEARANCE = 0.14
MAX_TRANSIT_Z = 0.82

ACTION_TARGETS = {
    "R1": ["SHELL_PICK", "WB1_PLACE"],
    "R2": ["RAIL_PICK_H", "RAIL_PLACE_H"],
    "R3": [
        "RAIL_PICK_A", "RAIL_PLACE_A", "RAIL_PICK_B", "RAIL_PLACE_B",
    ],
    "R4": [],
    "R5": ["PLC_PICK", "PLC_PLACE", "PSU_PICK", "PSU_PLACE"],
    "R6": [
        "SERVO_PICK", "SERVO_PLACE", "DMA_PICK", "DMA_PLACE",
        "CONTACTOR_PICK", "CONTACTOR_PLACE", "BREAKER_PICK",
        "BREAKER_PLACE",
    ],
    "R7": ["SCREW_1", "SCREW_2", "SCREW_3", "SCREW_4"],
    "R8": [],
}

# R4 and R8 are intentionally parked in the conveyor version of the cell.
# They remain available for a later inspection/rework operation, but no
# longer carry the complete cabinet between stations.
PARK_ONLY_STOWS = {
    "R4": {
        "joints": [-1.8683631960557017, -0.4111497735244001,
                   -0.5830902493505132, -0.576096014417379,
                   1.571382351849766, -1.7592756947317254],
        "position": [-1.85, 0.45, 0.48],
    },
    "R8": {
        "joints": [0.15417726611145444, -0.9097539174226865,
                   0.6367671568352853, 1.8435949404162377,
                   -1.5706422275267946, -0.21862780554855288],
        "position": [0.321, -0.152, 0.50],
    },
}

R1_MAX_DOWN_TILT = math.radians(5.0)

# R1 grasps only the shell's outer top frame.  Its parking/carry posture is
# the pick APP itself, so the loaded arm retracts vertically and never folds
# the cabinet back toward Link2.  The WB1 branch uses an unwrapped joint-1
# value to avoid an unnecessary full revolution across +/-pi.
R1_EDGE_STOW_POSITION = [-3.65, 1.3625, 0.482]
R1_EDGE_STOW_JOINTS = [
    -1.1107650778817013,
    0.45240051167724715,
    0.5119875004861754,
    0.6058427269398168,
    -1.5707460069523014,
    -0.3582462845362908,
]
R1_EDGE_PLACE_APP_JOINTS = [
    0.14082626904379048,
    -0.3781930728873828,
    -0.627367889514387,
    -0.564470114525633,
    1.57019662603859,
    -2.248222207073014,
]
R1_EDGE_ENDPOINTS = {
    "SHELL_PICK": {
        "app": list(R1_EDGE_STOW_JOINTS),
        "tcp": [
            -1.1091739822050268,
            0.3360480901995536,
            0.9546491811950999,
            0.28070961058456473,
            -1.5708484248763877,
            -0.35664342408819244,
        ],
    },
    "WB1_PLACE": {
        "app": list(R1_EDGE_PLACE_APP_JOINTS),
        "tcp": [
            0.13902175451438029,
            -0.27895983352164994,
            -1.032701975249649,
            -0.25963548961825184,
            1.5708359408433288,
            -2.2500392291217794,
        ],
    },
}

GRIPPER_CLOSED_GAPS = {
    # Calibrated against the shell's real outer-frame thickness.  At this
    # opening both R1 inner rubber pads touch with about a 1 mm contact band.
    "R1": 0.032,
    # The R3 vertical rails are 17.5 mm wide.  The slim 6 mm pads use a
    # 17 mm inner opening, giving about 0.5 mm bilateral compression while
    # still fitting between the rail and cabinet side wall.
    "R3": 0.017,
    # R4 grips the complete cabinet by its east upper frame.  A 19 mm
    # opening makes both inner rubber pads contact that frame at the paired
    # handoff/WB2 TCPs; the old generic 35 mm gap touched only one side.
    "R4": 0.019,
}
GRIPPER_FINGER_THICKNESSES = {"R3": 0.006}

R1_LOADED_PLACE_WAYPOINTS = [
    [-3.65, 1.3625, 0.52],
    [-3.35, 1.25, 0.52],
    [-3.10, 1.00, 0.52],
    [-3.02, 0.70, 0.52],
    [-3.05, 0.48, 0.52],
    [-3.15, 0.3625, 0.52],
    [-3.15, 0.3625, 0.482],
]

R2_MAX_DOWN_TILT = math.radians(5.0)
R2_MIN_TRANSIT_Z = 0.48
R2_ENDPOINT_POSITION_TOLERANCE = 0.0003
R2_RAIL_STOW_POSITION = [-3.76, -0.725, 0.5]
R2_RAIL_STOW_JOINTS = [
    -0.20124853874932302,
    1.4414846496717004,
    -0.910666299623198,
    1.0401141390062012,
    -1.5708276268472006,
    -1.7720373548216661,
]
R2_RAIL_PLACE_APP_JOINTS = [
    -2.194446313655045,
    1.4407925557796561,
    -0.9086187737698329,
    1.038754166412068,
    -1.5708848923830256,
    -3.765080708581119,
]
R2_RAIL_ENDPOINTS = {
    "RAIL_PICK_H": {
        "app": list(R2_RAIL_STOW_JOINTS),
        "tcp": [
            -0.20160510871278756,
            1.8734140901786847,
            -1.294024810943995,
            0.9914195503813313,
            -1.5708001922096808,
            -1.7723922842910147,
        ],
    },
    "RAIL_PLACE_H": {
        "app": list(R2_RAIL_PLACE_APP_JOINTS),
        "tcp": [
            -2.1953512890066538,
            1.8585511618193857,
            -1.2846887339950601,
            0.9969467506975576,
            -1.5708003155587358,
            -3.7659841064259445,
        ],
    },
}

# The far-left vertical rail needs a measured 5 degree reach compensation.
# Keep a one-degree numerical/path interpolation margin; all other R3 process
# poses remain essentially vertical.
R3_MAX_DOWN_TILT = math.radians(6.0)
R3_RAIL_STOW_POSITION = [-2.35, -0.76625, 0.48]
R3_RAIL_STOW_JOINTS = [
    -0.8196571254928204,
    -0.32062503936282916,
    -0.7188455186057693,
    -0.530800336859059,
    1.5707561866810895,
    -0.819631428010331,
]
R3_RAIL_ENDPOINTS = {
    "RAIL_PICK_A": {
        "app": list(R3_RAIL_STOW_JOINTS),
        "tcp": [-0.8214459722370293, -0.21087287363789153,
                -1.3878469488956502, 0.027230347721916903,
                1.5708618593327683, -0.8214665558068481],
    },
    "RAIL_PICK_B": {
        "app": [-1.0613092048503625, -0.06953685429382084,
                -1.1313727363831814, -0.456136368490196,
                1.5602106136036307, 2.129094348296684],
        "tcp": [-1.063678902649575, -0.01920066997346037,
                -1.5847025661356522, -0.053906107674310366,
                1.560120291056684, 2.126597733882388],
    },
    "RAIL_PLACE_A": {
        "app": [1.9706283058513678, -0.1669142851158898,
                -0.923078881650094, -0.48026812301288935,
                1.570754305441655, 1.970661135298693],
        "tcp": [1.9684929355765464, -0.08066279045663016,
                -1.5401601046272317, 0.049306659037770206,
                1.5708691236046253, 1.968458437051975],
    },
    "RAIL_PLACE_B": {
        "app": [2.46537589651843, -1.0933030643458812,
                0.5299543970838073, -0.9231110398492044,
                1.5480074841472897, -0.6289347027740508],
        "tcp": [2.4654919378500203, -0.5243630102780096,
                -0.9750485657908026, 0.012804565493868603,
                1.5479892505669994, -0.628818535027469],
    },
}

# Loaded R3 rail placement uses deterministic Cartesian corridors.  The
# wrist-only rotations happen at 0.50--0.52 m, well clear of the fixtures;
# OMPL is retained only as a local fallback inside a checked joint segment.
R3_RAIL_A_SOURCE_ROTATED = [
    -0.8190966640116017, -0.4129473046327794, -0.4411122045337928,
    -0.7156438741905164, 1.5709377366184167, 0.751649554687175,
]
R3_RAIL_A_TARGET_ROTATED = [
    1.9894076868149604, -0.19505895111657567, -0.756069324624015,
    -0.6196832455210135, 1.5706294701820713, 1.989442748321257,
]
R3_RAIL_B_TARGET_ROTATED = [
    -2.334489179651832, -0.20281665166956486, -0.8134346797377353,
    -0.5694839658239372, 1.4844460521423857, 0.8559956766891341,
]
R3_RAIL_PLACE_WAYPOINTS = {
    "RAIL_PLACE_A": [
        [-2.359104, -0.500272, 0.52],
        [-2.966574, 0.007440, 0.52],
        [-2.948450, 0.120000, 0.52],
    ],
    "RAIL_PLACE_B": [
        [-2.850000, -0.750000, 0.50],
        [-3.150000, -0.700000, 0.50],
        [-3.400000, -0.300000, 0.48],
        [-3.351450, 0.080000, 0.40],
    ],
}

# R5 (vacuum) approaches its device basket from the EAST with joint1 around
# 125 deg, mirroring the legacy R2 PCB-supply posture, so the elbow never
# sweeps through the basket's west wall on the way to the pick APP.
R5_DEVICE_STOW_POSITION = [-1.05, -0.40, 0.5]
R5_DEVICE_STOW_JOINTS = [
    2.1886201477683665,
    0.12217304763960307,
    1.6284110952568797,
    -0.1815143659609314,
    -1.5707963267948966,
    -2.1764007687754342,
]

# R4 uses the cabinet's east upper frame at both transfer stations.  These
# paired IK branches keep the flange down with the same tool yaw, so the
# rigidly carried cabinet does not rotate or snap when the release station
# pose is applied.  They are valid for the R4 base at (-1.50, 0.55).
R4_TRANSFER_ENDPOINTS = {
    "HANDOFF_PICK": {
        "app": [
            -1.8683631960557017,
            -0.4111497735244001,
            -0.5830902493505132,
            -0.576096014417379,
            1.571382351849766,
            -1.7592756947317254,
        ],
        "tcp": [
            -1.8697087268154862,
            -0.3072356523380217,
            -0.9941390973318518,
            -0.2695087048181048,
            1.5714252408848295,
            -1.7606336292819642,
        ],
    },
    "WB2_PLACE": {
        "app": [
            -0.1072393304800352,
            -0.21157664865300285,
            -0.8684277630071603,
            -0.49015222228162925,
            1.5707466701383597,
            0.0018422428751065922,
        ],
        "tcp": [
            -0.10869707503052842,
            -0.13840863748772286,
            -1.2102360214077186,
            -0.22191099329707797,
            1.570778880873187,
            0.0003718098551621853,
        ],
    },
}
R4_MAX_DOWN_TILT = math.radians(15.0)
R4_STOW_POSITION = [-1.85, 0.45, 0.48]

# R7 (screwdriver) parks at the first screw APP posture inherited from the
# legacy R4 screw poses (joint5 = 90 deg keeps the bit vertical).
R7_SCREW_STOW_POSITION = [-0.011, 0.385, 0.492]
R7_SCREW_STOW_JOINTS = [
    math.radians(-29.4),
    math.radians(-25.9),
    math.radians(-59.2),
    math.radians(-4.9),
    math.radians(90.0),
    math.radians(-29.4),
]

# Each R3 placement/transfer is a four-pose, down-facing corridor:
# high source -> high target -> APP -> TCP.  The high source/target pair
# sweeps around the base instead of cutting through its singular near field.
R3_FIXED_CORRIDORS = {
    "RAIL_PICK_A": [
        list(R3_RAIL_STOW_JOINTS),
        list(R3_RAIL_STOW_JOINTS),
        list(R3_RAIL_ENDPOINTS["RAIL_PICK_A"]["app"]),
        list(R3_RAIL_ENDPOINTS["RAIL_PICK_A"]["tcp"]),
    ],
    "RAIL_PICK_B": [
        list(R3_RAIL_STOW_JOINTS),
        list(R3_RAIL_STOW_JOINTS),
        list(R3_RAIL_ENDPOINTS["RAIL_PICK_B"]["app"]),
        list(R3_RAIL_ENDPOINTS["RAIL_PICK_B"]["tcp"]),
    ],
    "RAIL_PLACE_A": [
        [-0.2852678323744997, 0.09487285926813716, -1.198117014259878,
         -0.4669780675369237, 1.570677054949856, 1.285474503378845],
        [-1.6320583661542671, 0.842511621279276, -0.2784620887636603,
         -2.1347254967471097, 1.570722446187598, -0.061319486001825574],
        list(R3_RAIL_ENDPOINTS["RAIL_PLACE_A"]["app"]),
        list(R3_RAIL_ENDPOINTS["RAIL_PLACE_A"]["tcp"]),
    ],
    "RAIL_PLACE_B": [
        [-0.28569163199697867, 0.11384175639803118, -1.273254959343968,
         -0.4110476221415289, 1.5706927234998505, 1.2850436542417194],
        [-1.1094889787316262, 0.9514754357979198, -0.48084780255958987,
         -2.041274685937356, 1.5707183880178555, 0.46123570733332997],
        list(R3_RAIL_ENDPOINTS["RAIL_PLACE_B"]["app"]),
        list(R3_RAIL_ENDPOINTS["RAIL_PLACE_B"]["tcp"]),
    ],
    "WB1_PICK": [
        [-0.34384109671030494, -0.10438214215802777, -0.9960787968545954,
         -0.4698678537526164, 1.5707582055822638, 1.2267704488989413],
        [2.3812066780795105, -0.10324854486595081, -0.9986250016901213,
         -0.46835363161495724, 1.5707504675216892, 2.573415114411622],
        [2.347113908070501, -0.284039005675369, -0.7711865984337799,
         -0.5149832775033443, 1.5707512672977026, 2.539332975011823],
        [2.3452961305250817, -0.20583368204232855, -1.1256252315080542,
         -0.2400158378175452, 1.5708548585237763, 2.537489925381056],
    ],
    "HANDOFF_PLACE": [
        [-0.34384109671030494, -0.10438214215802777, -0.9960787968545954,
         -0.4698678537526164, 1.5707582055822638, 1.2267704488989413],
        [0.6995988100906141, -0.10423890066703656, -0.9964130565861407,
         -0.4696626420894354, 1.5707580708142022, 2.2702258625569094],
        [1.037311262565833, -0.4488134358109104, -0.5889744815664892,
         -0.5324831998976705, 1.5707557466066184, 1.229521213047228],
        [1.0358324542766972, -0.3728614662483145, -0.9000794547702932,
         -0.2984947023906299, 1.570848682509303, 1.228027341808474],
    ],
}
R3_FIXED_HIGH_Z = {
    "RAIL_PICK_A": 0.48,
    "RAIL_PICK_B": 0.44,
    "RAIL_PLACE_A": 0.48,
    "RAIL_PLACE_B": 0.46,
    "WB1_PICK": 0.48,
    "HANDOFF_PLACE": 0.46,
}
R3_HANDOFF_DETOUR = [
    [1.9231335746648615, -0.544891902544832, -0.42879419961151605,
     -0.5965580343073893, 1.5707524307929903, 2.115344889692814],
    [1.1335093177012912, -0.5459150093631511, -0.4265422804553055,
     -0.5978205653400974, 1.5707551573117415, 1.3257235108828542],
    [0.9306514635699328, -0.378096282195866, -0.7481200037663334,
     -0.44515009106243975, 1.570842273171948, 1.1228558689493813],
    [1.037311262565833, -0.4488134358109104, -0.5889744815664892,
     -0.5324831998976705, 1.5707557466066184, 1.229521213047228],
    [1.0358324542766972, -0.3728614662483145, -0.9000794547702932,
     -0.2984947023906299, 1.570848682509303, 1.228027341808474],
]

PARTS = {
    "shell": ("Shell_1", "REF_shell", "/FiveCR5A_Cell/Parts/Shell_Stack"),
    "rail_h": ("Rail_H1", "REF_rail_h1", "/FiveCR5A_Cell/Baskets/R2_Stand"),
    "rail_a": ("Rail_1", "REF_rail_v1", "/FiveCR5A_Cell/Baskets/R3_Rail_Rack"),
    "rail_b": ("Rail_2", "REF_rail_v2", "/FiveCR5A_Cell/Baskets/R3_Rail_Rack"),
    "plc": ("PLC_1", "REF_plc", "/FiveCR5A_Cell/Baskets/R5_Device_Basket"),
    "psu": ("PSU_1", "REF_psu", "/FiveCR5A_Cell/Baskets/R5_Device_Basket"),
    "servo": ("Servo_1", "REF_servo", "/FiveCR5A_Cell/Baskets/R6_Device_Basket"),
    "dma": ("DMA_1", "REF_dma", "/FiveCR5A_Cell/Baskets/R6_Device_Basket"),
    "contactor": ("Contactor_1", "REF_contactor", "/FiveCR5A_Cell/Baskets/R6_Device_Basket"),
    "breaker": ("Breaker_1", "REF_breaker", "/FiveCR5A_Cell/Baskets/R6_Device_Basket"),
}

STATIONS = {
    "wb1": [-3.15, 0.25, 0.27],
    "handoff": [-2.45, 0.40, 0.27],
    "wb2": [-1.20, 0.25, 0.27],
    "staging": [0.05, 0.25, 0.27],
    "output": [0.75, 0.25, 0.270],
}
R4_HANDOFF_CENTER = [-2.05, 0.45, 0.27]
REFERENCE_CENTER = [0.75, -0.10, 0.27]

PICK_PART = {
    ("R1", "SHELL_PICK"): "shell",
    ("R2", "RAIL_PICK_H"): "rail_h",
    ("R3", "RAIL_PICK_A"): "rail_a",
    ("R3", "RAIL_PICK_B"): "rail_b",
    ("R5", "PLC_PICK"): "plc",
    ("R5", "PSU_PICK"): "psu",
    ("R6", "SERVO_PICK"): "servo",
    ("R6", "DMA_PICK"): "dma",
    ("R6", "CONTACTOR_PICK"): "contactor",
    ("R6", "BREAKER_PICK"): "breaker",
}

PLACE_PART = {
    ("R1", "WB1_PLACE"): "shell",
    ("R2", "RAIL_PLACE_H"): "rail_h",
    ("R3", "RAIL_PLACE_A"): "rail_a",
    ("R3", "RAIL_PLACE_B"): "rail_b",
    ("R5", "PLC_PLACE"): "plc",
    ("R5", "PSU_PLACE"): "psu",
    ("R6", "SERVO_PLACE"): "servo",
    ("R6", "DMA_PLACE"): "dma",
    ("R6", "CONTACTOR_PLACE"): "contactor",
    ("R6", "BREAKER_PLACE"): "breaker",
}

SOURCE_FIXTURE = {
    "shell": "/FiveCR5A_Cell/Conveyors/Cabinet_Conveyor",
    "rail_h": "/FiveCR5A_Cell/Baskets/R2_Stand",
    "rail_a": "/FiveCR5A_Cell/Baskets/R3_Rail_Rack",
    "rail_b": "/FiveCR5A_Cell/Baskets/R3_Rail_Rack",
    "plc": "/FiveCR5A_Cell/Baskets/R5_Device_Basket",
    "psu": "/FiveCR5A_Cell/Baskets/R5_Device_Basket",
    "servo": "/FiveCR5A_Cell/Baskets/R6_Device_Basket",
    "dma": "/FiveCR5A_Cell/Baskets/R6_Device_Basket",
    "contactor": "/FiveCR5A_Cell/Baskets/R6_Device_Basket",
    "breaker": "/FiveCR5A_Cell/Baskets/R6_Device_Basket",
}

# Only these fixtures are ignored, and only during the final APP <-> TCP
# contact leg.  They remain obstacles during every high-level transit.
CONTACT_STATION = {
    ("R1", "WB1_PLACE"): "wb1",
    ("R2", "RAIL_PLACE_H"): "wb1",
    ("R3", "RAIL_PLACE_A"): "wb1",
    ("R3", "RAIL_PLACE_B"): "wb1",
    ("R5", "PLC_PLACE"): "wb2",
    ("R5", "PSU_PLACE"): "wb2",
    ("R6", "SERVO_PLACE"): "wb2",
    ("R6", "DMA_PLACE"): "wb2",
    ("R6", "CONTACTOR_PLACE"): "wb2",
    ("R6", "BREAKER_PLACE"): "wb2",
    ("R7", "SCREW_1"): "staging",
    ("R7", "SCREW_2"): "staging",
    ("R7", "SCREW_3"): "staging",
    ("R7", "SCREW_4"): "staging",
}

STATION_FIXTURE_PATHS = {
    "wb1": [
        f"{SCENE_ROOT}/Conveyors/Central_Indexing_Conveyor",
        f"{SCENE_ROOT}/Conveyors/Indexing_Pallet_1",
    ],
    "handoff": [f"{SCENE_ROOT}/Areas/Handoff_Area"],
    "wb2": [
        f"{SCENE_ROOT}/Conveyors/Central_Indexing_Conveyor",
        f"{SCENE_ROOT}/Conveyors/Indexing_Pallet_1",
    ],
    "staging": [
        f"{SCENE_ROOT}/Conveyors/Central_Indexing_Conveyor",
        f"{SCENE_ROOT}/Conveyors/Indexing_Pallet_1",
    ],
    "output": [
        f"{SCENE_ROOT}/Conveyors/Central_Indexing_Conveyor",
        f"{SCENE_ROOT}/Conveyors/Indexing_Pallet_1",
    ],
    "r2_stand": [f"{SCENE_ROOT}/Baskets/R2_Stand"],
    "r3_rack": [f"{SCENE_ROOT}/Baskets/R3_Rail_Rack"],
    "r5_basket": [f"{SCENE_ROOT}/Baskets/R5_Device_Basket"],
    "r6_basket": [f"{SCENE_ROOT}/Baskets/R6_Device_Basket"],
}

# Transfer stems whose APP -> TCP contact leg lands on a station fixture
# (the assembled cabinet rests on the fixture surface during planning too).
TRANSFER_STATION_FIXTURE = {
}

# Pick contact legs inside a shared source fixture: the suction plate /
# gripper legitimately overlaps adjacent parts and the container walls.
BASKET_PICK_CONTAINER = {
    ("R2", "RAIL_PICK_H"): "r2_stand",
    ("R3", "RAIL_PICK_A"): "r3_rack",
    ("R3", "RAIL_PICK_B"): "r3_rack",
    ("R5", "PLC_PICK"): "r5_basket",
    ("R5", "PSU_PICK"): "r5_basket",
    ("R6", "SERVO_PICK"): "r6_basket",
    ("R6", "DMA_PICK"): "r6_basket",
    ("R6", "CONTACTOR_PICK"): "r6_basket",
    ("R6", "BREAKER_PICK"): "r6_basket",
}
BASKET_PICK_MATES = {
    ("R2", "RAIL_PICK_H"): ["Door_Spare"],
    ("R3", "RAIL_PICK_A"): ["Rail_2"],
    ("R3", "RAIL_PICK_B"): ["Rail_1"],
    ("R5", "PLC_PICK"): ["PSU_1"],
    ("R5", "PSU_PICK"): ["PLC_1"],
    ("R6", "SERVO_PICK"): ["DMA_1", "Contactor_1", "Breaker_1"],
    ("R6", "DMA_PICK"): ["Servo_1", "Contactor_1", "Breaker_1"],
    ("R6", "CONTACTOR_PICK"): ["Servo_1", "DMA_1", "Breaker_1"],
    ("R6", "BREAKER_PICK"): ["Servo_1", "DMA_1", "Contactor_1"],
}


def fingerprint(path: Path) -> dict[str, object]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"size": path.stat().st_size, "sha256": digest.hexdigest()}


def quintic(value: float) -> float:
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def quaternion_multiply(first: list[float], second: list[float]) -> list[float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def rotate_vector(quaternion: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = quaternion
    rotated = quaternion_multiply(
        quaternion_multiply([x, y, z, w], [*vector, 0.0]),
        [-x, -y, -z, w],
    )
    return rotated[:3]


def compose_pose(first: list[float], second: list[float]) -> list[float]:
    translated = rotate_vector(first[3:], second[:3])
    return [first[index] + translated[index] for index in range(3)] + quaternion_multiply(
        first[3:], second[3:]
    )


def inverse_pose(pose: list[float]) -> list[float]:
    inverse_quaternion = [-pose[3], -pose[4], -pose[5], pose[6]]
    inverse_translation = rotate_vector(
        inverse_quaternion, [-pose[0], -pose[1], -pose[2]]
    )
    return inverse_translation + inverse_quaternion


def nearest_down_quaternion(current: list[float]) -> list[float]:
    """Project a tool quaternion onto the z-down family, preserving yaw."""
    norm = math.hypot(float(current[0]), float(current[1]))
    if norm < 1e-9:
        return list(DOWN_QUATERNION)
    return [float(current[0]) / norm, float(current[1]) / norm, 0.0, 0.0]


def quaternion_from_z_axis(direction: list[float]) -> list[float]:
    """Quaternion rotating local +Z onto ``direction``."""
    norm = math.sqrt(sum(float(value) ** 2 for value in direction))
    if norm < 1e-9:
        raise ValueError("tool TCP must be offset from the visible flange")
    tx, ty, tz = (float(value) / norm for value in direction)
    if tz < -1.0 + 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    quaternion = [-ty, tx, 0.0, 1.0 + tz]
    qnorm = math.sqrt(sum(value * value for value in quaternion))
    return [value / qnorm for value in quaternion]


def expand_keyframes(keyframes: list[list[float]], speed: float) -> tuple[list[list[float]], int]:
    """Return smooth frames and the index of the TCP keyframe."""
    frames = [list(keyframes[0])]
    tcp_frame = 0
    frame_delta = MAX_FRAME_DELTA * max(0.2, min(float(speed), 3.0))
    for leg_index, (start, end) in enumerate(zip(keyframes, keyframes[1:]), start=1):
        largest = max(abs(b - a) for a, b in zip(start, end))
        steps = max(
            8,
            int(math.ceil(largest * QUINTIC_MAX_SLOPE / frame_delta)),
        )
        for index in range(1, steps + 1):
            blend = quintic(index / steps)
            frames.append([a + (b - a) * blend for a, b in zip(start, end)])
        if leg_index == 2:
            tcp_frame = len(frames) - 1
    return frames, tcp_frame


def unique_alias(sim, root: int, alias: str) -> int:
    matches = [
        int(handle)
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
        if str(sim.getObjectAlias(handle, 0)) == alias
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected one object named {alias}, found {len(matches)}")
    return matches[0]


class Scene:
    def __init__(self, client: RemoteAPIClient):
        self.client = client
        self.sim = client.require("sim")
        self.ik = client.require("simIK")
        self.cell = int(self.sim.getObject(SCENE_ROOT))
        self.parts_parent = int(self.sim.getObject(f"{SCENE_ROOT}/Parts"))
        self.decorative_ground = [
            int(handle)
            for handle in self.sim.getObjectsInTree(
                self.cell, self.sim.object_shape_type, 0
            )
            if str(self.sim.getObjectAlias(handle, 0)).startswith(
                ("Floor_Zone_", "Floor_Path_", "Floor_Border_")
            )
        ]
        self.roots = {robot: int(self.sim.getObject(f"/{robot}")) for robot in ROBOT_IDS}
        self.joints = {}
        self.tips = {}
        self.collision_shapes: dict[str, list[int]] = {}
        self.fixed_base_shapes: dict[str, list[int]] = {}
        self.tool_roots: dict[str, int] = {}
        self.virt_tips: dict[str, int] = {}
        self.down_tips: dict[str, int] = {}
        self.real_tip_in_virtual: dict[str, list[float]] = {}
        stale_helpers = [
            int(handle)
            for handle in self.sim.getObjectsInTree(
                int(self.sim.handle_scene), self.sim.handle_all, 0
            )
            if str(self.sim.getObjectAlias(handle, 0))
            in {"Assembly_Collision_Planner", "Assembly_Runtime_Batch"}
        ]
        if stale_helpers:
            self.sim.removeObjects(stale_helpers)
        for robot in ROBOT_IDS:
            tree = self.sim.getObjectsInTree(self.roots[robot], self.sim.handle_all, 0)
            stale_virtual_tips = [
                int(handle)
                for handle in tree
                if str(self.sim.getObjectAlias(handle, 0)) in {
                    f"{robot}_assembly_virt_tip",
                    f"{robot}_assembly_down_tip",
                }
            ]
            if stale_virtual_tips:
                self.sim.removeObjects(stale_virtual_tips)
                tree = self.sim.getObjectsInTree(
                    self.roots[robot], self.sim.handle_all, 0
                )
            by_alias = {str(self.sim.getObjectAlias(h, 0)): int(h) for h in tree}
            self.joints[robot] = [by_alias[name] for name in ARM_JOINT_ALIASES]
            self.tips[robot] = by_alias[ROBOT_TIPS[robot]]
            self.tool_roots[robot] = by_alias[f"{robot}T"]
            self.collision_shapes[robot] = [
                int(handle)
                for handle in self.sim.getObjectsInTree(
                    self.roots[robot], self.sim.object_shape_type, 0
                )
                if "respondable" in str(self.sim.getObjectAlias(handle, 0)).lower()
                and "base_link" not in str(self.sim.getObjectAlias(handle, 0)).lower()
            ]
            self.fixed_base_shapes[robot] = [
                int(handle)
                for handle in self.sim.getObjectsInTree(
                    self.roots[robot], self.sim.object_shape_type, 0
                )
                if "respondable" in str(self.sim.getObjectAlias(handle, 0)).lower()
                and "base_link" in str(self.sim.getObjectAlias(handle, 0)).lower()
            ]
            # This CoppeliaSim build models the IK chain only through Link6;
            # a tip below the flange is silently treated as if its tool
            # offset did not exist.  Solve on a virtual Link6 tip and offset
            # its target so the visible TCP reaches the requested point.
            link6_visual = by_alias["Link6_visual"]
            virt = int(self.sim.createDummy(0.004))
            self.sim.setObjectAlias(virt, f"{robot}_assembly_virt_tip", {"aliasIndex": 0})
            self.sim.setObjectParent(virt, link6_visual, False)
            self.sim.setObjectPose(
                virt, link6_visual, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            )
            self.sim.setObjectInt32Param(virt, self.sim.objintparam_visibility_layer, 0)
            self.virt_tips[robot] = virt
            real_tip_pose = [
                float(value)
                for value in self.sim.getObjectPose(self.tips[robot], virt)
            ]
            down_tip = int(self.sim.createDummy(0.004))
            self.sim.setObjectAlias(
                down_tip, f"{robot}_assembly_down_tip", {"aliasIndex": 0}
            )
            self.sim.setObjectParent(down_tip, virt, False)
            self.sim.setObjectPose(
                down_tip,
                virt,
                real_tip_pose[:3]
                + quaternion_from_z_axis(real_tip_pose[:3]),
            )
            self.sim.setObjectInt32Param(
                down_tip, self.sim.objintparam_visibility_layer, 0
            )
            self.down_tips[robot] = down_tip
            self.real_tip_in_virtual[robot] = [
                float(value)
                for value in self.sim.getObjectPose(down_tip, virt)
            ]
        self._batch_script: int | None = None
        self._planner_script: int | None = None

    def by_alias(self, alias: str) -> int:
        return unique_alias(self.sim, self.cell, alias)

    def set_joints(self, robot: str, values: Iterable[float]) -> None:
        for handle, value in zip(self.joints[robot], values):
            self.sim.setJointPosition(handle, float(value))

    def set_all_home(self) -> None:
        for robot in ROBOT_IDS:
            self.set_joints(robot, HOME)

    def virtual_target_pose(
        self,
        robot: str,
        real_position: list[float],
        real_quaternion: list[float] | None = None,
    ) -> list[float]:
        desired_real_pose = [float(value) for value in real_position] + list(
            real_quaternion if real_quaternion is not None else DOWN_QUATERNION
        )
        return compose_pose(
            desired_real_pose,
            inverse_pose(self.real_tip_in_virtual[robot]),
        )

    def create_collision_pair(
        self,
        robot: str,
        exclusions: Iterable[int] = (),
        include_other_robots: bool = True,
        moving_exclusions: Iterable[int] = (),
    ) -> tuple[int, int]:
        """Collections for the moving chain versus fixtures and other arms."""
        moving = int(self.sim.createCollection(0))
        environment = int(self.sim.createCollection(0))
        moving_exclusion_shapes: set[int] = set()
        for handle in moving_exclusions:
            moving_exclusion_shapes.update(
                int(shape)
                for shape in self.sim.getObjectsInTree(
                    int(handle), self.sim.object_shape_type, 0
                )
            )
        if moving_exclusion_shapes:
            moving_shapes = set(self.collision_shapes[robot])
            moving_shapes.update(
                int(shape)
                for shape in self.sim.getObjectsInTree(
                    self.tool_roots[robot], self.sim.object_shape_type, 0
                )
            )
            for shape in moving_shapes - moving_exclusion_shapes:
                self.sim.addItemToCollection(
                    moving, self.sim.handle_single, shape, 0
                )
        else:
            self.add_robot_collision_geometry(moving, robot)
        self.sim.addItemToCollection(
            environment, self.sim.handle_tree, self.cell, 0
        )
        for handle in self.decorative_ground:
            self.sim.addItemToCollection(
                environment, self.sim.handle_single, handle, 1
            )
        for handle in exclusions:
            self.sim.addItemToCollection(
                environment, self.sim.handle_tree, int(handle), 1
            )
        # Do not compare Link1 with its own base mesh: the supplied CR5
        # collision model intentionally overlaps those adjacent bodies at the
        # joint, so that pair reports a collision in every valid posture.
        if include_other_robots:
            for other, root in self.roots.items():
                if other != robot:
                    self.add_robot_collision_geometry(environment, other)
                    for handle in self.fixed_base_shapes[other]:
                        self.sim.addItemToCollection(
                            environment, self.sim.handle_single, handle, 0
                        )
        return moving, environment

    def create_carried_object_collision_pair(
        self,
        robot: str,
        carried: int,
        exclusions: Iterable[int] = (),
    ) -> tuple[int, int]:
        """Check a carried workpiece separately from its gripping robot."""
        moving = int(self.sim.createCollection(0))
        environment = int(self.sim.createCollection(0))
        # Building the environment from explicit shapes is intentional.
        # CoppeliaSim collection subtraction is order/model-property
        # dependent: adding the complete Cell tree and later subtracting an
        # assembly child can leave that child's shapes in the collection.
        # That made a legal rail-to-shell placement contact look like an
        # obstacle.  Resolve every excluded subtree first, then never add
        # those shapes to the environment.
        excluded_shapes = set(self.decorative_ground)
        for handle in [int(carried), *[int(value) for value in exclusions]]:
            excluded_shapes.update(
                int(shape)
                for shape in self.sim.getObjectsInTree(
                    handle, self.sim.object_shape_type, 0
                )
            )
        self.sim.addItemToCollection(
            moving, self.sim.handle_tree, int(carried), 0
        )
        for shape in self.sim.getObjectsInTree(
            self.cell, self.sim.object_shape_type, 0
        ):
            if int(shape) not in excluded_shapes:
                self.sim.addItemToCollection(
                    environment, self.sim.handle_single, int(shape), 0
                )
        # Include the gripping arm links and base, but not its tool tree: tool
        # versus workpiece contact is the intended grasp.  Exclusions are
        # applied AFTER this inclusion so callers can also waive the
        # part-versus-own-arm checks (tight transit poses graze the links).
        for handle in self.collision_shapes[robot] + self.fixed_base_shapes[robot]:
            if int(handle) not in excluded_shapes:
                self.sim.addItemToCollection(
                    environment, self.sim.handle_single, handle, 0
                )
        for other in ROBOT_IDS:
            if other == robot:
                continue
            self.add_robot_collision_geometry(environment, other)
            for handle in self.fixed_base_shapes[other]:
                self.sim.addItemToCollection(
                    environment, self.sim.handle_single, handle, 0
                )
        return moving, environment

    def add_robot_collision_geometry(self, collection: int, robot: str) -> None:
        """Use physical response meshes, not duplicate visual meshes."""
        for shape in self.collision_shapes[robot]:
            self.sim.addItemToCollection(
                collection, self.sim.handle_single, shape, 0
            )
        # Tool geometry and anything attached below its tip are dynamic tree
        # members, so a carried rail/cabinet participates automatically.
        self.sim.addItemToCollection(
            collection, self.sim.handle_tree, self.tool_roots[robot], 0
        )

    def destroy_collision_pair(self, pair: tuple[int, int]) -> None:
        for collection in pair:
            self.sim.destroyCollection(collection)

    def collision_details(self, pair: tuple[int, int]) -> tuple[str, str] | None:
        result, handles = self.sim.checkCollision(*pair)
        if int(result) <= 0:
            return None
        return (
            str(self.sim.getObjectAlias(int(handles[0]), 1)),
            str(self.sim.getObjectAlias(int(handles[1]), 1)),
        )

    def install_planner_script(self) -> None:
        if self._planner_script is not None:
            return
        code = """
sim=require 'sim'
simOMPL=require 'simOMPL'

function planJointPathImpl(joints,tip,movingCollections,environmentCollections,startState,goalState,maxTilt,minTipZ,maxTime)
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
    -- The wrist-branch transition can hide a sharp flange-tilt spike between
    -- otherwise valid nodes.  A 0.1% joint-space resolution catches that
    -- spike before OMPL returns a path that later fails dense replay.
    simOMPL.setStateValidityCheckingResolution(task,0.001)
    local cosLimit=math.cos(maxTilt)
    local function stateValid(state)
        for i=1,#joints,1 do sim.setJointPosition(joints[i],state[i]) end
        for i=1,#movingCollections,1 do
            if sim.checkCollision(movingCollections[i],environmentCollections[i])>0 then return false end
        end
        if maxTilt<3.0 then
            local m=sim.getObjectMatrix(tip,sim.handle_world)
            if m[11]>-cosLimit then return false end
        end
        if minTipZ>-9.0 then
            local p=sim.getObjectPosition(tip,sim.handle_world)
            if p[3]<minTipZ then return false end
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

function validateJointPath(joints,tip,movingCollections,environmentCollections,flatPath,maxTilt,minTipZ)
    local saved={}
    for i=1,#joints,1 do saved[i]=sim.getJointPosition(joints[i]) end
    local cosLimit=math.cos(maxTilt)
    local maxObservedTilt=0.0
    local minObservedZ=1e9
    for offset=1,#flatPath,#joints do
        for i=1,#joints,1 do sim.setJointPosition(joints[i],flatPath[offset+i-1]) end
        local collision=0
        local pair={-1,-1}
        for i=1,#movingCollections,1 do
            local result,hit=sim.checkCollision(movingCollections[i],environmentCollections[i])
            if result>0 then collision=result pair=hit break end
        end
        local m=sim.getObjectMatrix(tip,sim.handle_world)
        local downCos=math.max(-1.0,math.min(1.0,-m[11]))
        local tilt=math.acos(downCos)
        local p=sim.getObjectPosition(tip,sim.handle_world)
        maxObservedTilt=math.max(maxObservedTilt,tilt)
        minObservedZ=math.min(minObservedZ,p[3])
        if collision>0 or tilt>maxTilt or (minTipZ>-9.0 and p[3]<minTipZ) then
            local first=-1
            local second=-1
            if collision>0 then first=pair[1] second=pair[2] end
            for i=1,#joints,1 do sim.setJointPosition(joints[i],saved[i]) end
            return false,math.floor((offset-1)/#joints),maxObservedTilt,minObservedZ,first,second
        end
    end
    for i=1,#joints,1 do sim.setJointPosition(joints[i],saved[i]) end
    return true,-1,maxObservedTilt,minObservedZ,-1,-1
end
"""
        self._planner_script = int(
            self.sim.createScript(self.sim.scripttype_customization, code, 0)
        )
        self.sim.setObjectAlias(self._planner_script, "Assembly_Collision_Planner")
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
        min_tip_z: float = -10.0,
        include_other_robots: bool = True,
        additional_pairs: Iterable[tuple[int, int]] = (),
        moving_exclusions: Iterable[int] = (),
        max_time: float = 6.0,
    ) -> list[list[float]]:
        self.install_planner_script()
        pair = self.create_collision_pair(
            robot,
            exclusions,
            include_other_robots=include_other_robots,
            moving_exclusions=moving_exclusions,
        )
        try:
            pairs = [pair, *list(additional_pairs)]
            solved, flat, planner_error = self.sim.callScriptFunction(
                "planJointPath", self._planner_script, self.joints[robot],
                self.down_tips[robot],
                [value[0] for value in pairs],
                [value[1] for value in pairs],
                start, goal,
                float(max_tilt), float(min_tip_z), float(max_time),
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

    def install_batch_script(self) -> None:
        if self._batch_script is not None:
            return
        code = """
function applyFrame(joints, values, robotCollections, environmentCollections, spinHandle, spinAngle)
    for i=1,#joints,1 do sim.setJointPosition(joints[i],values[i]) end
    if spinHandle >= 0 then
        local p=sim.getObjectParent(spinHandle)
        local o=sim.getObjectOrientation(spinHandle,p)
        o[3]=spinAngle
        sim.setObjectOrientation(spinHandle,p,o)
    end
    local hits={}
    for i=1,#robotCollections,1 do
        local result,pair=sim.checkCollision(robotCollections[i],environmentCollections[i])
        if result>0 then
            hits[#hits+1]=pair[1]
            hits[#hits+1]=pair[2]
        end
    end
    return hits
end
"""
        self._batch_script = int(self.sim.createScript(self.sim.scripttype_customization, code, 0))
        self.sim.setObjectAlias(self._batch_script, "Assembly_Runtime_Batch")
        self.sim.initScript(self._batch_script)

    def remove_batch_script(self) -> None:
        if self._batch_script is not None:
            try:
                self.sim.removeObjects([self._batch_script])
            finally:
                self._batch_script = None

    def apply_frame(
        self,
        positions: dict[str, list[float]],
        collision_pairs: list[tuple[int, int]],
        spin: tuple[int, float] | None = None,
    ) -> None:
        if self._batch_script is None:
            raise RuntimeError("batch script is not installed")
        joint_handles: list[int] = []
        values: list[float] = []
        for robot, joints in positions.items():
            joint_handles.extend(self.joints[robot])
            values.extend(joints)
        spin_handle, spin_angle = spin if spin is not None else (-1, 0.0)
        hits = self.sim.callScriptFunction(
            "applyFrame", self._batch_script, joint_handles, values,
            [pair[0] for pair in collision_pairs],
            [pair[1] for pair in collision_pairs],
            int(spin_handle), float(spin_angle),
        )
        if hits:
            names = []
            for index in range(0, len(hits), 2):
                names.append(
                    f"{self.sim.getObjectAlias(int(hits[index]), 1)} <-> "
                    f"{self.sim.getObjectAlias(int(hits[index + 1]), 1)}"
                )
            raise RuntimeError("collision detected: " + ", ".join(names))


def normalize_config(values: Iterable[float]) -> list[float]:
    normalized = []
    for index, value in enumerate(values):
        angle = float(value)
        if index != 2:
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        normalized.append(angle)
    return normalized


def unwrap_config_near(reference: list[float], values: Iterable[float]) -> list[float]:
    """Keep cyclic joints on the nearest numeric branch to ``reference``."""
    unwrapped = []
    for index, (anchor, value) in enumerate(zip(reference, values)):
        angle = float(value)
        if index != 2:
            angle += round((float(anchor) - angle) / (2.0 * math.pi)) * (
                2.0 * math.pi
            )
        unwrapped.append(angle)
    return unwrapped


def config_distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def densify_path(path: list[list[float]]) -> list[list[float]]:
    dense = [list(path[0])]
    for start, end in zip(path, path[1:]):
        largest = max(abs(b - a) for a, b in zip(start, end))
        # Smoothstep's peak slope is 1.875, so using only total_delta/limit
        # underestimates the largest adjacent frame near the segment middle.
        steps = max(
            1,
            int(math.ceil(largest * QUINTIC_MAX_SLOPE / MAX_FRAME_DELTA)),
        )
        for index in range(1, steps + 1):
            blend = quintic(index / steps)
            dense.append([a + (b - a) * blend for a, b in zip(start, end)])
    return dense


def ik_candidates(
    scene: Scene,
    robot: str,
    position: list[float],
    seed: list[float],
    exclusions: Iterable[int] = (),
    attempts: int = 48,
    max_solutions: int = 8,
    fixed_quaternion: list[float] | None = None,
) -> list[list[float]]:
    """Find several down-facing IK branches and reject colliding branches."""
    sim, ik = scene.sim, scene.ik
    target = int(sim.createDummy(0.01))
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    scene.set_joints(robot, seed)
    seed_down = (
        [float(value) for value in fixed_quaternion]
        if fixed_quaternion is not None
        else nearest_down_quaternion(
            [
                float(value)
                for value in sim.getObjectQuaternion(scene.down_tips[robot], -1)
            ]
        )
    )
    sim.setObjectPose(
        target, -1, scene.virtual_target_pose(robot, position, seed_down)
    )
    environment = int(ik.createEnvironment())
    group = int(ik.createGroup(environment))
    pair = scene.create_collision_pair(robot, exclusions)
    try:
        ik.setGroupCalculation(environment, group, ik.method_damped_least_squares, 0.20, 250)
        element, _, _ = ik.addElementFromScene(
            environment, group, scene.roots[robot], scene.virt_tips[robot], target,
            ik.constraint_pose,
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
            desired_down = (
                [float(value) for value in fixed_quaternion]
                if fixed_quaternion is not None
                else nearest_down_quaternion(
                    [
                        float(value)
                        for value in sim.getObjectQuaternion(
                            scene.down_tips[robot], -1
                        )
                    ]
                )
            )
            target_pose = scene.virtual_target_pose(
                robot, position, desired_down
            )
            actual_position: list[float] = []
            for _ in range(6):
                sim.setObjectPose(target, -1, target_pose)
                result = ik.handleGroup(
                    environment, group, {"syncWorlds": True}
                )
                actual_position = [
                    float(value)
                    for value in sim.getObjectPosition(scene.tips[robot], -1)
                ]
                error = [
                    desired - actual
                    for desired, actual in zip(position, actual_position)
                ]
                if math.sqrt(sum(value * value for value in error)) <= IK_POSITION_TOLERANCE:
                    break
                for axis in range(3):
                    target_pose[axis] += error[axis]
            solution = normalize_config(
                float(sim.getJointPosition(handle)) for handle in scene.joints[robot]
            )
            scene.set_joints(robot, solution)
            if scene.collision_details(pair) is not None:
                continue
            if math.dist(actual_position, position) > IK_POSITION_TOLERANCE:
                continue
            matrix = sim.getObjectMatrix(scene.down_tips[robot], -1)
            if float(matrix[10]) > -math.cos(MAX_DOWN_TILT):
                continue
            if all(config_distance(solution, known) > 0.10 for known in solutions):
                solutions.append(solution)
            if len(solutions) >= max_solutions:
                break
        solutions.sort(key=lambda value: config_distance(value, seed))
        return solutions
    finally:
        scene.destroy_collision_pair(pair)
        ik.eraseEnvironment(environment)
        sim.removeObjects([target])
        scene.set_joints(robot, seed)


def cartesian_down_line(
    scene: Scene,
    robot: str,
    start_config: list[float],
    start_position: list[float],
    end_position: list[float],
    exclusions: Iterable[int],
    position_tolerance: float = IK_POSITION_TOLERANCE,
    fixed_quaternion: list[float] | None = None,
    max_tilt: float = MAX_DOWN_TILT,
    additional_pairs: Iterable[tuple[int, int]] = (),
    moving_exclusions: Iterable[int] = (),
) -> list[list[float]]:
    """IK-sample a straight tool-down approach and collision-check every sample."""
    sim, ik = scene.sim, scene.ik
    scene.set_joints(robot, start_config)
    desired_down = (
        [float(value) for value in fixed_quaternion]
        if fixed_quaternion is not None
        else nearest_down_quaternion(
            [
                float(value)
                for value in sim.getObjectQuaternion(scene.down_tips[robot], -1)
            ]
        )
    )
    target = int(sim.createDummy(0.01))
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    environment = int(ik.createEnvironment())
    group = int(ik.createGroup(environment))
    pair = scene.create_collision_pair(
        robot, exclusions, moving_exclusions=moving_exclusions
    )
    pairs = [pair, *list(additional_pairs)]
    distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(start_position, end_position)))
    steps = max(8, int(math.ceil(distance / 0.012)))
    path = [list(start_config)]
    try:
        ik.setGroupCalculation(environment, group, ik.method_damped_least_squares, 0.18, 300)
        element, _, _ = ik.addElementFromScene(
            environment, group, scene.roots[robot], scene.virt_tips[robot], target,
            ik.constraint_pose,
        )
        ik.setElementPrecision(
            environment, group, int(element),
            [position_tolerance, IK_ANGLE_TOLERANCE],
        )
        for index in range(1, steps + 1):
            ratio = index / steps
            position = [a + (b - a) * ratio for a, b in zip(start_position, end_position)]
            target_pose = scene.virtual_target_pose(
                robot, position, desired_down
            )
            actual_position: list[float] = []
            result = None
            # simIK reports the residual of the virtual Link6 tip.  On the
            # supplied CR5 model that can still leave the visible tool TCP
            # several centimetres from the requested point.  Close that gap
            # using measured visible-TCP feedback instead of accepting the
            # solver return code alone.
            for _ in range(6):
                sim.setObjectPose(target, -1, target_pose)
                result = ik.handleGroup(
                    environment, group, {"syncWorlds": True}
                )
                code = int(
                    result[0]
                    if isinstance(result, (tuple, list))
                    else result
                )
                residual = (
                    result[2]
                    if isinstance(result, (tuple, list)) and len(result) > 2
                    else []
                )
                within_explicit_tolerance = (
                    len(residual) >= 2
                    and float(residual[0]) <= position_tolerance
                    and float(residual[1]) <= IK_ANGLE_TOLERANCE
                )
                actual_position = [
                    float(value)
                    for value in sim.getObjectPosition(scene.tips[robot], -1)
                ]
                error = [
                    desired - actual
                    for desired, actual in zip(position, actual_position)
                ]
                if math.sqrt(sum(value * value for value in error)) <= position_tolerance:
                    break
                for axis in range(3):
                    target_pose[axis] += error[axis]
            if result is None:
                raise RuntimeError(f"Cartesian IK produced no result for {robot}")
            if not actual_position:
                raise RuntimeError(
                    f"Cartesian IK failed for {robot} at {position}: {result}"
                )
            config = unwrap_config_near(
                path[-1],
                (sim.getJointPosition(handle) for handle in scene.joints[robot]),
            )
            scene.set_joints(robot, config)
            actual_position = [
                float(value)
                for value in sim.getObjectPosition(scene.tips[robot], -1)
            ]
            if math.dist(actual_position, position) > position_tolerance:
                raise RuntimeError(
                    f"{robot} visible TCP residual exceeded tolerance at {position}"
                )
            for active_pair in pairs:
                collision = scene.collision_details(active_pair)
                if collision is not None:
                    raise RuntimeError(
                        f"{robot} Cartesian collision at {position}: {collision}"
                    )
            matrix = sim.getObjectMatrix(scene.down_tips[robot], -1)
            if float(matrix[10]) > -math.cos(max_tilt):
                raise RuntimeError(
                    f"{robot} flange tilt exceeded "
                    f"{math.degrees(max_tilt):.0f} degrees"
                )
            path.append(config)
        return path
    finally:
        scene.destroy_collision_pair(pair)
        ik.eraseEnvironment(environment)
        sim.removeObjects([target])


def cartesian_down_route(
    scene: Scene,
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
        if math.dist(position, waypoint) <= 1e-6:
            continue
        segment = cartesian_down_line(
            scene, robot, config, position, waypoint, exclusions,
            position_tolerance=TRANSIT_POSITION_TOLERANCE,
        )
        path.extend(segment[1:])
        config = path[-1]
        position = list(waypoint)
    return path


def collision_checked_joint_route(
    scene: Scene,
    robot: str,
    start: list[float],
    goal: list[float],
    attempts: int = 240,
) -> list[list[float]]:
    """Find a checked one/two-leg route for the one-time zero->stow move."""
    pair = scene.create_collision_pair(robot)

    def valid_segment(left: list[float], right: list[float]) -> list[list[float]] | None:
        frames = densify_path([left, right])
        for frame in frames:
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
        raise RuntimeError(f"no collision-free zero-to-stow route for {robot}")
    finally:
        scene.destroy_collision_pair(pair)
        scene.set_joints(robot, start)


def validated_joint_path(
    scene: Scene,
    robot: str,
    frames: list[list[float]],
    exclusions: Iterable[int],
    max_tilt: float = MAX_DOWN_TILT,
    min_tip_z: float = -10.0,
    include_other_robots: bool = True,
    additional_pairs: Iterable[tuple[int, int]] = (),
    moving_exclusions: Iterable[int] = (),
) -> list[list[float]]:
    """Validate an already sampled joint path against all active geometry."""
    pair = scene.create_collision_pair(
        robot,
        exclusions,
        include_other_robots=include_other_robots,
        moving_exclusions=moving_exclusions,
    )
    try:
        scene.install_planner_script()
        pairs = [pair, *list(additional_pairs)]
        flat = [value for frame in frames for value in frame]
        valid, index, observed_tilt, observed_z, first, second = (
            scene.sim.callScriptFunction(
                "validateJointPath",
                scene._planner_script,
                scene.joints[robot],
                scene.down_tips[robot],
                [value[0] for value in pairs],
                [value[1] for value in pairs],
                flat,
                float(max_tilt),
                float(min_tip_z),
            )
        )
        if not valid:
            collision = ""
            if int(first) >= 0 and int(second) >= 0:
                collision = (
                    f", collision={scene.sim.getObjectAlias(int(first), 1)} <-> "
                    f"{scene.sim.getObjectAlias(int(second), 1)}"
                )
            raise RuntimeError(
                f"{robot} joint corridor invalid at frame {int(index)}: "
                f"max_tilt={math.degrees(float(observed_tilt)):.1f} deg, "
                f"min_z={float(observed_z):.3f}{collision}"
            )
        return frames
    finally:
        scene.destroy_collision_pair(pair)


def validated_joint_line(
    scene: Scene,
    robot: str,
    start: list[float],
    goal: list[float],
    exclusions: Iterable[int],
    max_tilt: float = MAX_DOWN_TILT,
    min_tip_z: float = -10.0,
    include_other_robots: bool = True,
    additional_pairs: Iterable[tuple[int, int]] = (),
    moving_exclusions: Iterable[int] = (),
) -> list[list[float]]:
    """Densify a known process branch and validate collision + tool tilt."""
    return validated_joint_path(
        scene,
        robot,
        densify_path([start, goal]),
        exclusions,
        max_tilt=max_tilt,
        min_tip_z=min_tip_z,
        include_other_robots=include_other_robots,
        additional_pairs=additional_pairs,
        moving_exclusions=moving_exclusions,
    )


def collision_aware_joint_segment(
    scene: Scene,
    robot: str,
    start: list[float],
    goal: list[float],
    exclusions: Iterable[int],
    *,
    max_tilt: float,
    min_tip_z: float,
    additional_pairs: Iterable[tuple[int, int]] = (),
    moving_exclusions: Iterable[int] = (),
    include_other_robots: bool = True,
    ompl_max_time: float = 6.0,
) -> list[list[float]]:
    """Use a direct joint line when safe, otherwise search with loaded geometry."""
    extra = list(additional_pairs)
    try:
        return validated_joint_line(
            scene,
            robot,
            start,
            goal,
            exclusions,
            max_tilt=max_tilt,
            min_tip_z=min_tip_z,
            additional_pairs=extra,
            moving_exclusions=moving_exclusions,
            include_other_robots=include_other_robots,
        )
    except RuntimeError:
        raw = scene.ompl_path(
            robot,
            start,
            goal,
            exclusions,
            max_tilt=max_tilt,
            min_tip_z=min_tip_z,
            additional_pairs=extra,
            moving_exclusions=moving_exclusions,
            include_other_robots=include_other_robots,
            max_time=ompl_max_time,
        )
        continuous = [list(start)]
        for config in raw:
            candidate = unwrap_config_near(continuous[-1], config)
            if config_distance(continuous[-1], candidate) > 1e-7:
                continuous.append(candidate)
        resolved_goal = unwrap_config_near(continuous[-1], goal)
        if config_distance(continuous[-1], resolved_goal) > 1e-7:
            continuous.append(resolved_goal)
        dense = densify_path(continuous)
        return validated_joint_path(
            scene,
            robot,
            dense,
            exclusions,
            max_tilt=max_tilt,
            min_tip_z=min_tip_z,
            additional_pairs=extra,
            moving_exclusions=moving_exclusions,
            include_other_robots=include_other_robots,
        )


def attach_part_for_loaded_planning(
    scene: Scene,
    robot: str,
    part_key: str,
    pick_config: list[float],
) -> tuple[int, int, list[float], int, int, int, list[float], list[float]]:
    """Attach a real workpiece at its measured pick transform for planning."""
    part = scene.by_alias(PARTS[part_key][0])
    parent = int(scene.sim.getObjectParent(part))
    world_matrix = [
        float(value) for value in scene.sim.getObjectMatrix(part, -1)
    ]
    tool = scene.tool_roots[robot]
    left = unique_alias(scene.sim, scene.roots[robot], f"{robot}T_left_finger_link")
    right = unique_alias(scene.sim, scene.roots[robot], f"{robot}T_right_finger_link")
    left_position = list(scene.sim.getObjectPosition(left, tool))
    right_position = list(scene.sim.getObjectPosition(right, tool))
    closed_gap = GRIPPER_CLOSED_GAPS.get(robot, 0.035)
    finger_thickness = GRIPPER_FINGER_THICKNESSES.get(robot, 0.020)
    closed_y = (closed_gap + finger_thickness) / 2.0
    closed_left = list(left_position)
    closed_right = list(right_position)
    closed_left[1] = closed_y
    closed_right[1] = -closed_y
    scene.sim.setObjectPosition(left, tool, closed_left)
    scene.sim.setObjectPosition(right, tool, closed_right)
    scene.set_joints(robot, pick_config)
    scene.sim.setObjectParent(part, scene.tips[robot], True)
    return (
        part, parent, world_matrix,
        tool, left, right, left_position, right_position,
    )


def restore_part_after_loaded_planning(
    scene: Scene,
    state: tuple[
        int, int, list[float], int, int, int, list[float], list[float]
    ],
) -> None:
    (
        part, parent, world_matrix, tool,
        left, right, left_position, right_position,
    ) = state
    scene.sim.setObjectParent(part, parent, False)
    scene.sim.setObjectMatrix(part, -1, world_matrix)
    scene.sim.setObjectPosition(left, tool, left_position)
    scene.sim.setObjectPosition(right, tool, right_position)


def r3_fixed_down_action(
    scene: Scene,
    stem: str,
    stow: list[float],
    app_position: list[float],
    tcp_position: list[float],
    transit_exclusions: list[int],
    contact_exclusions: list[int],
) -> dict[str, object]:
    """Validate R3's deterministic high arc and edge-contact micro-drop."""
    high_source, high_target, app_config, tcp_config = R3_FIXED_CORRIDORS[stem]
    high_z = R3_FIXED_HIGH_Z[stem]
    assembly_stow = R3_FIXED_CORRIDORS["WB1_PICK"][1]
    rail_pick_key = PICK_PART.get(("R3", stem))
    rail_place_key = PLACE_PART.get(("R3", stem))
    rail_key = rail_pick_key or rail_place_key

    def checked_segment(
        left: list[float],
        right: list[float],
        exclusions: list[int],
        min_z: float,
        *,
        contact: bool = False,
        loaded_key: str | None = rail_place_key,
    ) -> list[list[float]]:
        if loaded_key is None:
            return collision_aware_joint_segment(
                scene,
                "R3",
                left,
                right,
                exclusions,
                max_tilt=R3_MAX_DOWN_TILT,
                min_tip_z=min_z,
            )
        pick_stem = "RAIL_PICK_A" if loaded_key == "rail_a" else "RAIL_PICK_B"
        attachment = attach_part_for_loaded_planning(
            scene,
            "R3",
            loaded_key,
            list(R3_RAIL_ENDPOINTS[pick_stem]["tcp"]),
        )
        carried_exclusions = contact_exclusions if contact else []
        if contact and rail_pick_key is not None:
            carried_exclusions = [scene.by_alias("R3_Rail_Rack_Floor")]
        carried_pair = scene.create_carried_object_collision_pair(
            "R3",
            attachment[0],
            carried_exclusions,
        )
        try:
            return collision_aware_joint_segment(
                scene,
                "R3",
                left,
                right,
                exclusions,
                max_tilt=R3_MAX_DOWN_TILT,
                min_tip_z=min_z,
                additional_pairs=[carried_pair],
                moving_exclusions=[attachment[0]],
            )
        finally:
            scene.destroy_collision_pair(carried_pair)
            restore_part_after_loaded_planning(scene, attachment)

    def checked_cartesian(
        start_config: list[float],
        start_position: list[float],
        end_position: list[float],
        exclusions: list[int],
        *,
        fixed_quaternion: list[float] | None = None,
        loaded_key: str | None = rail_place_key,
        contact: bool = False,
        position_tolerance: float = TRANSIT_POSITION_TOLERANCE,
    ) -> list[list[float]]:
        attachment: tuple[
            int, int, list[float], int, int, int, list[float], list[float]
        ] | None = None
        carried_pair: tuple[int, int] | None = None
        if loaded_key is not None:
            pick_stem = (
                "RAIL_PICK_A" if loaded_key == "rail_a" else "RAIL_PICK_B"
            )
            attachment = attach_part_for_loaded_planning(
                scene,
                "R3",
                loaded_key,
                list(R3_RAIL_ENDPOINTS[pick_stem]["tcp"]),
            )
            carried_exclusions = contact_exclusions if contact else []
            if contact and rail_pick_key is not None:
                carried_exclusions = [scene.by_alias("R3_Rail_Rack_Floor")]
            carried_pair = scene.create_carried_object_collision_pair(
                "R3",
                attachment[0],
                carried_exclusions,
            )
        try:
            return cartesian_down_line(
                scene,
                "R3",
                start_config,
                start_position,
                end_position,
                exclusions,
                position_tolerance=position_tolerance,
                fixed_quaternion=fixed_quaternion,
                max_tilt=R3_MAX_DOWN_TILT,
                additional_pairs=[carried_pair] if carried_pair is not None else [],
                moving_exclusions=[attachment[0]] if attachment is not None else [],
            )
        finally:
            if carried_pair is not None:
                scene.destroy_collision_pair(carried_pair)
            if attachment is not None:
                restore_part_after_loaded_planning(scene, attachment)

    if stem in R3_RAIL_PLACE_WAYPOINTS:
        def append_segment(
            path: list[list[float]], segment: list[list[float]]
        ) -> None:
            path.extend(segment if not path else segment[1:])

        start_config = (
            list(stow)
            if stem == "RAIL_PLACE_A"
            else list(R3_RAIL_ENDPOINTS["RAIL_PICK_B"]["app"])
        )
        scene.set_joints("R3", start_config)
        start_position = [
            float(value)
            for value in scene.sim.getObjectPosition(scene.tips["R3"], -1)
        ]
        source_quaternion = [
            float(value)
            for value in scene.sim.getObjectQuaternion(scene.down_tips["R3"], -1)
        ]
        lift_z = 0.52 if stem == "RAIL_PLACE_A" else 0.50
        lift_position = [start_position[0], start_position[1], lift_z]
        segments: list[list[list[float]]] = []
        lift_segment = checked_cartesian(
            start_config,
            start_position,
            lift_position,
            transit_exclusions,
            fixed_quaternion=source_quaternion,
        )
        segments.append(lift_segment)
        config = list(lift_segment[-1])
        position = list(lift_position)

        if stem == "RAIL_PLACE_A":
            rotate_source = checked_segment(
                config,
                R3_RAIL_A_SOURCE_ROTATED,
                transit_exclusions,
                0.505,
            )
            segments.append(rotate_source)
            config = list(rotate_source[-1])
            scene.set_joints("R3", config)
            transport_quaternion = [
                float(value)
                for value in scene.sim.getObjectQuaternion(
                    scene.down_tips["R3"], -1
                )
            ]
            for waypoint in R3_RAIL_PLACE_WAYPOINTS[stem]:
                segment = checked_cartesian(
                    config,
                    position,
                    waypoint,
                    transit_exclusions,
                    fixed_quaternion=transport_quaternion,
                )
                segments.append(segment)
                config, position = list(segment[-1]), list(waypoint)
            rotate_target = checked_segment(
                config,
                R3_RAIL_A_TARGET_ROTATED,
                transit_exclusions,
                0.505,
            )
            segments.append(rotate_target)
            config = list(rotate_target[-1])
            target_quaternion = source_quaternion
            app_target = [app_position[0], app_position[1], app_position[2]]
            app_segment = checked_cartesian(
                config,
                position,
                app_target,
                transit_exclusions,
                fixed_quaternion=target_quaternion,
                position_tolerance=IK_POSITION_TOLERANCE,
            )
            segments.append(app_segment)
            config, position = list(app_segment[-1]), app_target
        else:
            # B keeps its source orientation along the south side of the
            # base, then performs one short wrist rotation at the second
            # 0.50 m waypoint before approaching the cabinet.
            for waypoint in R3_RAIL_PLACE_WAYPOINTS[stem][:2]:
                segment = checked_cartesian(
                    config,
                    position,
                    waypoint,
                    transit_exclusions,
                    fixed_quaternion=source_quaternion,
                )
                segments.append(segment)
                config, position = list(segment[-1]), list(waypoint)
            rotate_target = checked_segment(
                config,
                R3_RAIL_B_TARGET_ROTATED,
                transit_exclusions,
                0.485,
            )
            segments.append(rotate_target)
            config = list(rotate_target[-1])
            scene.set_joints("R3", config)
            target_quaternion = [
                float(value)
                for value in scene.sim.getObjectQuaternion(
                    scene.down_tips["R3"], -1
                )
            ]
            waypoint = R3_RAIL_PLACE_WAYPOINTS[stem][2]
            segment = checked_cartesian(
                config,
                position,
                waypoint,
                transit_exclusions,
                fixed_quaternion=target_quaternion,
            )
            segments.append(segment)
            config, position = list(segment[-1]), list(waypoint)
            app_target = list(R3_RAIL_PLACE_WAYPOINTS[stem][3])
            app_segment = checked_cartesian(
                config,
                position,
                app_target,
                contact_exclusions,
                fixed_quaternion=target_quaternion,
                contact=True,
            )
            segments.append(app_segment)
            config, position = list(app_segment[-1]), app_target

        resolved_app_config = list(config)
        tcp_segment = checked_cartesian(
            config,
            position,
            tcp_position,
            contact_exclusions,
            fixed_quaternion=target_quaternion,
            contact=True,
            position_tolerance=IK_POSITION_TOLERANCE,
        )
        segments.append(tcp_segment)
        inbound: list[list[float]] = []
        for segment in segments:
            append_segment(inbound, segment)
        inbound = densify_path(inbound)
        tcp_frame = len(inbound) - 1
        resolved_tcp_config = list(inbound[-1])
        frames = inbound + list(reversed(inbound[:-1]))
        if stem == "RAIL_PLACE_B":
            empty_tail = checked_segment(
                start_config,
                stow,
                transit_exclusions,
                app_position[2] - 0.005,
                loaded_key=None,
            )
            frames.extend(empty_tail[1:])
        scene.set_joints("R3", stow)
        return {
            "frames": frames,
            "tcp_frame": tcp_frame,
            "endpoint_seeds": {
                "app": resolved_app_config,
                "tcp": resolved_tcp_config,
            },
            "transit_exclusions": [int(value) for value in transit_exclusions],
            "contact_exclusions": [int(value) for value in contact_exclusions],
            "clearance": {
                "preferred_z": lift_z,
                "safe_z": lift_z,
                "app_z": app_position[2],
                "tcp_z": tcp_position[2],
                "planner": "R3 deterministic loaded-rail Cartesian corridor",
                "transfer_tilt_limit_deg": math.degrees(R3_MAX_DOWN_TILT),
                "release_offset_y_m": abs(tcp_position[1] - 0.135),
                "rule": (
                    "vertical lift; fixed high waypoints; high wrist rotation; "
                    "vertical outside-wall release"
                ),
            },
        }

    if stem == "HANDOFF_PLACE":
        detour_configs = [assembly_stow] + R3_HANDOFF_DETOUR
        detour_min_z = [0.455, 0.455, 0.435, 0.435, tcp_position[2] - 0.005]
        segments = []
        for index, (left, right) in enumerate(
            zip(detour_configs, detour_configs[1:])
        ):
            exclusions = (
                contact_exclusions
                if index == len(detour_min_z) - 1
                else transit_exclusions
            )
            segments.append(
                checked_segment(
                    left,
                    right,
                    exclusions,
                    detour_min_z[index],
                    contact=index == len(detour_min_z) - 1,
                )
            )
    else:
        segments = [
            checked_segment(
                stow, high_source, transit_exclusions, high_z - 0.005,
            ),
            checked_segment(
                high_source, high_target, transit_exclusions, high_z - 0.005,
            ),
            checked_segment(
                high_target,
                app_config,
                transit_exclusions,
                app_position[2] - 0.005,
            ),
        ]
        if stem == "RAIL_PLACE_B":
            scene.set_joints("R3", app_config)
            held_quaternion = [
                float(value)
                for value in scene.sim.getObjectQuaternion(
                    scene.down_tips["R3"], -1
                )
            ]
            segments.append(
                checked_cartesian(
                    app_config,
                    app_position,
                    tcp_position,
                    contact_exclusions,
                    fixed_quaternion=held_quaternion,
                    contact=True,
                )
            )
        else:
            segments.append(
                checked_segment(
                    app_config,
                    tcp_config,
                    contact_exclusions,
                    tcp_position[2] - 0.005,
                    contact=True,
                )
            )
    inbound: list[list[float]] = []
    for segment in segments:
        inbound.extend(segment if not inbound else segment[1:])

    max_drop_xy = 0.0
    for config in segments[-1]:
        scene.set_joints("R3", config)
        actual = scene.sim.getObjectPosition(scene.tips["R3"], -1)
        max_drop_xy = max(
            max_drop_xy,
            math.hypot(
                float(actual[0]) - tcp_position[0],
                float(actual[1]) - tcp_position[1],
            ),
        )
    if max_drop_xy > TRANSIT_POSITION_TOLERANCE:
        raise RuntimeError(
            f"R3_{stem} edge-contact drop drifted {max_drop_xy:.3f} m laterally"
        )
    resolved_tcp_config = list(segments[-1][-1])
    for label, config, target in (
        ("APP", app_config, app_position),
        ("TCP", resolved_tcp_config, tcp_position),
    ):
        scene.set_joints("R3", config)
        actual = scene.sim.getObjectPosition(scene.tips["R3"], -1)
        if math.dist(actual, target) > TRANSIT_POSITION_TOLERANCE:
            raise RuntimeError(f"R3_{stem}_{label} residual exceeded tolerance")

    inbound = densify_path(inbound)
    tcp_frame = len(inbound) - 1
    if stem == "WB1_PICK":
        # Loaded cabinet stops at the WB1-side high pose instead of returning
        # through the rail rack parking volume.  ``inbound`` has been
        # densified again above, so pre-densification segment lengths cannot
        # identify this frame reliably.  Resolve the actual high-target frame
        # by configuration distance to keep the following HANDOFF action
        # exactly continuous.
        high_target_frame = min(
            range(tcp_frame + 1),
            key=lambda index: config_distance(inbound[index], high_target),
        )
        frames = inbound + list(
            reversed(inbound[high_target_frame:-1])
        )
    elif stem == "HANDOFF_PLACE":
        # Start where WB1_PICK ended.  Once the cabinet is released, the
        # empty arm may safely return through the rail-side high pose.
        outbound = list(reversed(inbound[:-1]))
        return_via = R3_FIXED_CORRIDORS["WB1_PICK"][0]
        tail_1 = validated_joint_line(
            scene, "R3", assembly_stow, return_via, contact_exclusions,
            max_tilt=R3_MAX_DOWN_TILT, min_tip_z=high_z - 0.005,
        )
        tail_2 = validated_joint_line(
            scene, "R3", return_via, stow, contact_exclusions,
            max_tilt=R3_MAX_DOWN_TILT, min_tip_z=high_z - 0.005,
        )
        frames = inbound + outbound + tail_1[1:] + tail_2[1:]
        frames = densify_path(frames)
    elif rail_pick_key is not None:
        if stem == "RAIL_PICK_B":
            scene.set_joints("R3", resolved_tcp_config)
            held_quaternion = [
                float(value)
                for value in scene.sim.getObjectQuaternion(
                    scene.down_tips["R3"], -1
                )
            ]
            first_outbound = checked_cartesian(
                resolved_tcp_config,
                tcp_position,
                app_position,
                contact_exclusions,
                fixed_quaternion=held_quaternion,
                loaded_key=rail_pick_key,
                contact=True,
            )
        else:
            first_outbound = checked_segment(
                resolved_tcp_config,
                app_config,
                contact_exclusions,
                tcp_position[2] - 0.005,
                loaded_key=rail_pick_key,
                contact=True,
            )
        loaded_outbound_segments = [first_outbound]
        if stem == "RAIL_PICK_B":
            loaded_outbound_segments.append(
                checked_segment(
                    list(first_outbound[-1]),
                    app_config,
                    contact_exclusions,
                    app_position[2] - TRANSIT_POSITION_TOLERANCE,
                    contact=True,
                    loaded_key=rail_pick_key,
                )
            )
        else:
            loaded_outbound_segments.extend([
                checked_segment(
                    app_config,
                    high_target,
                    transit_exclusions,
                    app_position[2] - 0.005,
                    loaded_key=rail_pick_key,
                ),
                checked_segment(
                    high_target,
                    high_source,
                    transit_exclusions,
                    high_z - 0.005,
                    loaded_key=rail_pick_key,
                ),
                checked_segment(
                    high_source,
                    stow,
                    transit_exclusions,
                    high_z - 0.005,
                    loaded_key=rail_pick_key,
                ),
            ])
        outbound: list[list[float]] = []
        for segment in loaded_outbound_segments:
            outbound.extend(segment if not outbound else segment[1:])
        frames = inbound + densify_path(outbound)[1:]
    else:
        frames = inbound + list(reversed(inbound[:-1]))
    scene.set_joints("R3", stow)
    return {
        "frames": frames,
        "tcp_frame": tcp_frame,
        "endpoint_seeds": {
            "app": list(app_config),
            "tcp": list(resolved_tcp_config),
        },
        "transit_exclusions": [int(value) for value in transit_exclusions],
        "contact_exclusions": [int(value) for value in contact_exclusions],
        "clearance": {
            "preferred_z": high_z,
            "safe_z": high_z,
            "app_z": app_position[2],
            "tcp_z": tcp_position[2],
            "planner": (
                "R3 load-aware OMPL/high arc and edge-contact drop"
                if rail_key is not None
                else "R3 fixed 5deg base arc and edge-contact drop"
            ),
            "transfer_tilt_limit_deg": math.degrees(R3_MAX_DOWN_TILT),
            "max_contact_xy_drift_m": max_drop_xy,
            "rule": "high arc; only the final APP-to-TCP micro-drop enters contact",
        },
    }


def action_exclusions(scene: Scene, robot: str, stem: str) -> tuple[list[int], list[int]]:
    """Exclude only the workpiece that the tool intentionally contacts."""
    pick_key = PICK_PART.get((robot, stem))
    key = pick_key or PLACE_PART.get((robot, stem))
    transit: list[int] = []
    contact: list[int] = []
    if key is not None:
        # Search the WHOLE scene: a part attached to a robot tip lives
        # outside the FiveCR5A_Cell subtree after its first pickup.
        part = unique_alias(
            scene.sim, int(scene.sim.handle_scene), PARTS[key][0]
        )
        transit.append(part)
        contact.append(part)
    # Pick/place contact legs may intentionally touch their source or target
    # station fixture.  Exclude that fixture only from APP <-> TCP; all high
    # transit frames keep it as a strict obstacle.  This applies to ordinary
    # part placement as well as whole-cabinet transfers.
    station = CONTACT_STATION.get((robot, stem))
    if station is not None:
        for path in STATION_FIXTURE_PATHS[station]:
            try:
                contact.append(int(scene.sim.getObject(path)))
            except Exception:
                continue
    container = BASKET_PICK_CONTAINER.get((robot, stem))
    if container is not None:
        for path in STATION_FIXTURE_PATHS[container]:
            try:
                # the lift out of the basket legitimately sweeps the walls
                transit.append(int(scene.sim.getObject(path)))
                contact.append(int(scene.sim.getObject(path)))
            except Exception:
                continue
        for alias in BASKET_PICK_MATES.get((robot, stem), []):
            try:
                handle = unique_alias(
                    scene.sim, int(scene.sim.handle_scene), alias
                )
                transit.append(handle)
                contact.append(handle)
            except RuntimeError:
                continue
    return list(dict.fromkeys(transit)), list(dict.fromkeys(contact))


def stow_position_candidates(
    scene: Scene, robot: str, first_app_position: list[float]
) -> list[list[float]]:
    """High parking points biased away from the source fixture.

    Parking directly above a raised bin can put Link5/Link6 through the top
    part even though the TCP itself is high.  The first candidates retract
    toward the robot base and then fan sideways before trying the raw APP
    projection.
    """
    base = [float(v) for v in scene.sim.getObjectPosition(scene.roots[robot], -1)]
    dx = first_app_position[0] - base[0]
    dy = first_app_position[1] - base[1]
    radius = max(math.hypot(dx, dy), 1e-6)
    tangent = [-dy / radius, dx / radius]
    xy_candidates: list[list[float]] = []
    for fraction in (0.58, 0.72):
        center = [base[0] + fraction * dx, base[1] + fraction * dy]
        xy_candidates.append(center)
        for side in (-1.0, 1.0):
            xy_candidates.append(
                [
                    center[0] + side * 0.12 * tangent[0],
                    center[1] + side * 0.12 * tangent[1],
                ]
            )
    xy_candidates.append(first_app_position[:2])
    heights = (
        PREFERRED_TRANSIT_Z,
        PREFERRED_TRANSIT_Z + 0.05,
        MIN_TRANSIT_Z,
    )
    return [[xy[0], xy[1], z] for xy in xy_candidates for z in heights]


def search_fixed_corridor(
    scene: Scene,
    robot: str,
    stem: str,
    stow: list[float],
    tcp_position: list[float],
    app_position: list[float],
    transit_exclusions: list[int],
    contact_exclusions: list[int],
) -> dict[str, list[float]] | None:
    """Four-pose down-facing corridor: high source -> high target -> APP ->
    TCP, with every leg collision/tilt validated.  Mirrors the legacy
    per-robot key-pose scheme (each pose solved down-facing, legs checked)."""
    sim = scene.sim
    base = [float(v) for v in sim.getObjectPosition(scene.roots[robot], -1)]
    dx = app_position[0] - base[0]
    dy = app_position[1] - base[1]
    radius = max(math.hypot(dx, dy), 1e-6)
    tangent = [-dy / radius, dx / radius]
    # high-source candidates arc east/north of the base at corridor height
    high_sources: list[list[float]] = []
    for angle in (-2.8, -2.4, -2.0, -1.6, -1.2, -0.6):
        hs = [
            base[0] + 0.28 * math.cos(angle),
            base[1] + 0.28 * math.sin(angle),
            PREFERRED_TRANSIT_Z,
        ]
        if math.hypot(hs[0] - app_position[0], hs[1] - app_position[1]) > 0.25:
            high_sources.append(hs)
    for source in high_sources:
        source_solutions = ik_candidates(
            scene, robot, source, stow, [], attempts=16, max_solutions=1
        )
        for source_config in source_solutions:
            try:
                validated_joint_line(
                    scene, robot, stow, source_config, transit_exclusions
                )
            except RuntimeError:
                continue
            # high target: pull toward the APP at corridor height
            toward = [app_position[0] - source[0], app_position[1] - source[1]]
            for fraction in (0.55, 0.75):
                target = [
                    source[0] + toward[0] * fraction,
                    source[1] + toward[1] * fraction,
                    PREFERRED_TRANSIT_Z,
                ]
                target_solutions = ik_candidates(
                    scene, robot, target, source_config, [], attempts=16, max_solutions=1
                )
                for target_config in target_solutions:
                    try:
                        validated_joint_line(
                            scene, robot, source_config, target_config, transit_exclusions
                        )
                    except RuntimeError:
                        continue
                    app_solutions = ik_candidates(
                        scene, robot, app_position, target_config,
                        contact_exclusions, attempts=16, max_solutions=1,
                    )
                    for app_config in app_solutions:
                        tcp_solutions = ik_candidates(
                            scene, robot, tcp_position, app_config,
                            contact_exclusions, attempts=16, max_solutions=1,
                        )
                        for tcp_config in tcp_solutions:
                            try:
                                validated_joint_line(
                                    scene, robot, target_config, app_config,
                                    transit_exclusions, min_tip_z=app_position[2] - 0.005,
                                )
                                validated_joint_line(
                                    scene, robot, app_config, tcp_config,
                                    contact_exclusions, min_tip_z=tcp_position[2] - 0.005,
                                    max_tilt=MAX_DOWN_TILT,
                                )
                            except RuntimeError:
                                continue
                            scene.set_joints(robot, stow)
                            return {
                                "source": [float(v) for v in source_config],
                                "target": [float(v) for v in target_config],
                                "app": [float(v) for v in app_config],
                                "tcp": [float(v) for v in tcp_config],
                            }
    scene.set_joints(robot, stow)
    return None


def _fixed_corridor_action(
    scene: Scene,
    robot: str,
    stem: str,
    stow: list[float],
    corridor: dict[str, list[float]],
    app_position: list[float],
    tcp_position: list[float],
    transit_exclusions: list[int],
    contact_exclusions: list[int],
) -> dict[str, object]:
    """Assemble the standard action dict from a found fixed corridor."""
    inbound = densify_path(
        [stow, corridor["source"], corridor["target"],
         corridor["app"], corridor["tcp"]]
    )
    outbound = densify_path(
        [corridor["tcp"], corridor["app"], corridor["target"],
         corridor["source"], stow]
    )
    scene.set_joints(robot, stow)
    return {
        "frames": inbound + outbound[1:],
        "tcp_frame": len(inbound) - 1,
        "endpoint_seeds": {
            "app": corridor["app"],
            "tcp": corridor["tcp"],
        },
        "transit_exclusions": [int(v) for v in transit_exclusions],
        "contact_exclusions": [int(v) for v in contact_exclusions],
        "clearance": {
            "preferred_z": PREFERRED_TRANSIT_Z,
            "safe_z": PREFERRED_TRANSIT_Z,
            "app_z": app_position[2],
            "tcp_z": tcp_position[2],
            "planner": "fixed_corridor",
            "transfer_tilt_limit_deg": math.degrees(MAX_TRANSFER_TILT),
            "rule": "fixed four-pose corridor; descent only on APP-to-TCP contact leg",
        },
    }


def r1_loaded_place_action(
    scene: Scene,
    stow: list[float],
    app_position: list[float],
    tcp_position: list[float],
    transit_exclusions: list[int],
    contact_exclusions: list[int],
) -> dict[str, object]:
    """Carry the shell around R1's base without sweeping it through Link2."""
    transit = cartesian_down_route(
        scene,
        "R1",
        stow,
        list(R1_EDGE_STOW_POSITION),
        [list(point) for point in R1_LOADED_PLACE_WAYPOINTS],
        transit_exclusions,
    )
    approach = cartesian_down_line(
        scene,
        "R1",
        transit[-1],
        app_position,
        tcp_position,
        contact_exclusions,
    )
    inbound = densify_path(transit + approach[1:])
    tcp_frame = len(inbound) - 1
    frames = inbound + list(reversed(inbound[:-1]))
    scene.set_joints("R1", stow)
    return {
        "frames": frames,
        "tcp_frame": tcp_frame,
        "endpoint_seeds": {
            "app": list(transit[-1]),
            "tcp": list(approach[-1]),
        },
        "transit_exclusions": [int(value) for value in transit_exclusions],
        "contact_exclusions": [int(value) for value in contact_exclusions],
        "clearance": {
            "preferred_z": 0.52,
            "safe_z": 0.52,
            "app_z": app_position[2],
            "tcp_z": tcp_position[2],
            "planner": "R1 loaded east-side Cartesian base arc",
            "transfer_tilt_limit_deg": math.degrees(R1_MAX_DOWN_TILT),
            "rule": (
                "shell remains on the east-side 0.52 m arc; WB1 fixture is "
                "excluded only on the final APP-to-TCP contact leg"
            ),
        },
    }


def r4_fixed_down_action(
    scene: Scene,
    stem: str,
    stow: list[float],
    app_position: list[float],
    tcp_position: list[float],
    transit_exclusions: list[int],
    contact_exclusions: list[int],
) -> dict[str, object]:
    """Use one physical grasp transform for R4's complete-cabinet transfer.

    The generic planner independently selected the two endpoint yaw families.
    That left the cabinet 429 mm from WB2 before the release-time snap.  R4
    instead uses same-branch +90-degree wrist solutions at both stations,
    keeps the visible flange within 15 degrees of down, and descends
    vertically at WB2.  The resulting frames are fixed for runtime replay.
    """
    sim = scene.sim
    scene.set_joints("R4", stow)
    fixed_quaternion = [
        float(value)
        for value in sim.getObjectQuaternion(scene.down_tips["R4"], -1)
    ]

    if stem == "HANDOFF_PICK":
        expected_app = list(R4_STOW_POSITION)
        actual_app = [
            float(value)
            for value in sim.getObjectPosition(scene.tips["R4"], -1)
        ]
        if math.dist(actual_app, expected_app) > IK_POSITION_TOLERANCE:
            raise RuntimeError(
                f"R4 handoff APP residual is "
                f"{math.dist(actual_app, expected_app):.4f} m"
            )
        approach = cartesian_down_line(
            scene,
            "R4",
            stow,
            expected_app,
            tcp_position,
            contact_exclusions,
            fixed_quaternion=fixed_quaternion,
            max_tilt=R4_MAX_DOWN_TILT,
        )
        inbound = densify_path(approach)
        planner = "R4 vertical bilateral frame pick"
        safe_z = expected_app[2]
    else:
        wb2_app = list(R4_TRANSFER_ENDPOINTS["WB2_PLACE"]["app"])
        wb2_tcp = list(R4_TRANSFER_ENDPOINTS["WB2_PLACE"]["tcp"])
        transit = validated_joint_line(
            scene,
            "R4",
            stow,
            wb2_app,
            transit_exclusions,
            max_tilt=R4_MAX_DOWN_TILT,
            min_tip_z=R4_STOW_POSITION[2] - 0.005,
        )
        approach = validated_joint_line(
            scene,
            "R4",
            wb2_app,
            wb2_tcp,
            contact_exclusions,
            max_tilt=R4_MAX_DOWN_TILT,
            min_tip_z=tcp_position[2] - 0.005,
        )
        inbound = densify_path(transit + approach[1:])
        planner = "R4 base-offset fixed same-branch joint corridor"
        safe_z = R4_STOW_POSITION[2]

    tcp_frame = len(inbound) - 1
    frames = inbound + list(reversed(inbound[:-1]))
    scene.set_joints("R4", stow)
    return {
        "frames": frames,
        "tcp_frame": tcp_frame,
        "endpoint_seeds": {
            "app": [
                float(value)
                for value in (
                    stow if stem == "HANDOFF_PICK" else transit[-1]
                )
            ],
            "tcp": [float(value) for value in approach[-1]],
        },
        "transit_exclusions": [int(value) for value in transit_exclusions],
        "contact_exclusions": [int(value) for value in contact_exclusions],
        "clearance": {
            "preferred_z": safe_z,
            "safe_z": safe_z,
            "app_z": app_position[2],
            "tcp_z": tcp_position[2],
            "planner": planner,
            "transfer_tilt_limit_deg": math.degrees(R4_MAX_DOWN_TILT),
            "gripper_closed_gap_m": GRIPPER_CLOSED_GAPS["R4"],
            "rule": (
                "vertical pick/lift; same-wrist-branch 15deg corridor; vertical "
                "WB2 placement with one preserved physical grasp transform"
            ),
        },
    }


def plan_action(
    scene: Scene,
    robot: str,
    stem: str,
    stow: list[float],
    stow_position: list[float],
    legacy_keyframes: list[list[float]] | None = None,
) -> dict[str, object]:
    sim = scene.sim
    app_handle = int(sim.getObject(POINTS[f"{robot}_{stem}_APP"]))
    tcp_handle = int(sim.getObject(POINTS[f"{robot}_{stem}_TCP"]))
    app_position = [float(v) for v in sim.getObjectPosition(app_handle, -1)]
    tcp_position = [float(v) for v in sim.getObjectPosition(tcp_handle, -1)]
    transit_exclusions, contact_exclusions = action_exclusions(scene, robot, stem)
    if robot == "R1" and stem == "WB1_PLACE":
        return r1_loaded_place_action(
            scene,
            stow,
            app_position,
            tcp_position,
            transit_exclusions,
            contact_exclusions,
        )
    if robot == "R4" and stem in R4_TRANSFER_ENDPOINTS:
        return r4_fixed_down_action(
            scene,
            stem,
            stow,
            app_position,
            tcp_position,
            transit_exclusions,
            contact_exclusions,
        )
    if robot in {"R6", "R8"}:
        corridor = search_fixed_corridor(
            scene, robot, stem, stow, tcp_position, app_position,
            transit_exclusions, contact_exclusions,
        )
        if corridor is not None:
            print(
                f"[corridor] {robot}_{stem} fixed four-pose corridor",
                flush=True,
            )
            return _fixed_corridor_action(
                scene, robot, stem, stow, corridor,
                app_position, tcp_position,
                transit_exclusions, contact_exclusions,
            )
    # Cross-station motion is never allowed to cut through the low work
    # zone.  The only descent below this corridor is the local APP -> TCP
    # contact leg, where the destination fixture is narrowly excluded.
    preferred_z = min(
        MAX_TRANSIT_Z,
        max(PREFERRED_TRANSIT_Z, app_position[2]),
    )
    safe_z_candidates = []
    candidate_z = preferred_z
    while candidate_z >= max(MIN_TRANSIT_Z, app_position[2]) - 1e-9:
        safe_z_candidates.append(round(candidate_z, 6))
        candidate_z -= 0.02
    minimum_reachable_z = max(MIN_TRANSIT_Z, app_position[2])
    if not safe_z_candidates or safe_z_candidates[-1] > minimum_reachable_z + 1e-6:
        safe_z_candidates.append(minimum_reachable_z)

    legacy_app: list[float] | None = None
    legacy_tcp: list[float] | None = None
    if legacy_keyframes is not None and len(legacy_keyframes) >= 3:
        legacy_app = [float(value) for value in legacy_keyframes[1]]
        legacy_tcp = [float(value) for value in legacy_keyframes[2]]
    if robot == "R4" and stem in R4_TRANSFER_ENDPOINTS:
        legacy_app = list(R4_TRANSFER_ENDPOINTS[stem]["app"])
        legacy_tcp = list(R4_TRANSFER_ENDPOINTS[stem]["tcp"])
    if robot == "R1" and stem in R1_EDGE_ENDPOINTS:
        legacy_app = list(R1_EDGE_ENDPOINTS[stem]["app"])
        legacy_tcp = list(R1_EDGE_ENDPOINTS[stem]["tcp"])
    if robot == "R2" and stem in R2_RAIL_ENDPOINTS:
        legacy_app = list(R2_RAIL_ENDPOINTS[stem]["app"])
        legacy_tcp = list(R2_RAIL_ENDPOINTS[stem]["tcp"])
    if robot == "R3" and stem in R3_RAIL_ENDPOINTS:
        legacy_app = list(R3_RAIL_ENDPOINTS[stem]["app"])
        legacy_tcp = list(R3_RAIL_ENDPOINTS[stem]["tcp"])

    # A scene-height change invalidates the old endpoint angles even when the
    # old elbow branch and wrist yaw are still valuable seeds.  Resolve a new
    # APP -> TCP branch at the generated dummy positions before planning the
    # cross-station transit.  This keeps the flange down and prevents a blind
    # joint-space interpolation into the raised fixture.
    endpoint_app: list[float] | None = None
    endpoint_approach: list[list[float]] | None = None
    endpoint_source = "cartesian from transit"
    if legacy_app is not None and legacy_tcp is not None:
        scene.set_joints(robot, legacy_app)
        app_error = math.dist(
            sim.getObjectPosition(scene.tips[robot], -1), app_position
        )
        scene.set_joints(robot, legacy_tcp)
        tcp_error = math.dist(
            sim.getObjectPosition(scene.tips[robot], -1), tcp_position
        )
        if app_error <= 0.004 and tcp_error <= 0.004:
            try:
                validated_joint_line(
                    scene, robot, legacy_app, legacy_app, transit_exclusions
                )
                if robot == "R2":
                    endpoint_approach = cartesian_down_line(
                        scene,
                        robot,
                        legacy_app,
                        app_position,
                        tcp_position,
                        contact_exclusions,
                        position_tolerance=R2_ENDPOINT_POSITION_TOLERANCE,
                    )
                else:
                    endpoint_approach = validated_joint_line(
                        scene, robot, legacy_app, legacy_tcp, contact_exclusions
                    )
                endpoint_app = legacy_app
                endpoint_source = "validated existing endpoint"
            except RuntimeError:
                endpoint_app = None
                endpoint_approach = None
        if endpoint_app is None:
            endpoint_errors: list[str] = []
            app_solutions = ik_candidates(
                scene,
                robot,
                app_position,
                legacy_app,
                transit_exclusions,
                attempts=32,
                max_solutions=6,
            )
            for candidate in app_solutions:
                try:
                    approach = cartesian_down_line(
                        scene,
                        robot,
                        candidate,
                        app_position,
                        tcp_position,
                        contact_exclusions,
                    )
                    endpoint_app = candidate
                    endpoint_approach = approach
                    endpoint_source = "raised endpoint from legacy branch seed"
                    break
                except RuntimeError as exc:
                    endpoint_errors.append(str(exc))
                    scene.set_joints(robot, stow)
            if endpoint_app is None or endpoint_approach is None:
                if robot in {"R6", "R8"}:
                    corridor = search_fixed_corridor(
                        scene, robot, stem, stow, tcp_position, app_position,
                        transit_exclusions, contact_exclusions,
                    )
                    if corridor is not None:
                        print(
                            f"[corridor] {robot}_{stem} fixed corridor "
                            f"(endpoint fallback)", flush=True,
                        )
                        return _fixed_corridor_action(
                            scene, robot, stem, stow, corridor,
                            app_position, tcp_position,
                            transit_exclusions, contact_exclusions,
                        )
                detail = " | ".join(endpoint_errors[-4:])
                raise RuntimeError(
                    f"unable to resolve down-facing endpoint for {robot}_{stem}"
                    + (f": {detail}" if detail else "")
                )
        scene.set_joints(robot, stow)

    def finish_transit(
        transit: list[list[float]], safe_z: float, method: str
    ) -> dict[str, object]:
        app_config = transit[-1]
        resolved_endpoint_used = False
        selected_endpoint_source = "continuous route APP branch"
        try:
            approach = cartesian_down_line(
                scene,
                robot,
                app_config,
                app_position,
                tcp_position,
                contact_exclusions,
                position_tolerance=(
                    R2_ENDPOINT_POSITION_TOLERANCE
                    if robot == "R2"
                    else IK_POSITION_TOLERANCE
                ),
            )
        except RuntimeError:
            if endpoint_app is None or endpoint_approach is None:
                raise
            app_snap = validated_joint_line(
                scene, robot, app_config, endpoint_app, transit_exclusions
            )
            transit.extend(app_snap[1:])
            app_config = endpoint_app
            approach = endpoint_approach
            resolved_endpoint_used = True
            selected_endpoint_source = endpoint_source
        inbound = densify_path(transit + approach[1:])
        tcp_index = len(inbound) - 1
        outbound_waypoints = list(reversed(approach)) + list(reversed(transit[:-1]))
        outbound = densify_path(outbound_waypoints)
        frames = inbound + outbound[1:]
        scene.set_joints(robot, stow)
        return {
            "frames": frames,
            "tcp_frame": tcp_index,
            "endpoint_seeds": {
                "app": [float(value) for value in app_config],
                "tcp": [float(value) for value in approach[-1]],
            },
            "transit_exclusions": [int(v) for v in transit_exclusions],
            "contact_exclusions": [int(v) for v in contact_exclusions],
            "clearance": {
                "preferred_z": preferred_z,
                "safe_z": safe_z,
                "app_z": app_position[2],
                "tcp_z": tcp_position[2],
                "planner": method,
                "transfer_tilt_limit_deg": math.degrees(
                    R1_MAX_DOWN_TILT
                    if robot == "R1"
                    else R2_MAX_DOWN_TILT
                    if robot == "R2"
                    else R3_MAX_DOWN_TILT
                )
                if robot in {"R1", "R2", "R3"}
                else (
                    math.degrees(MAX_TRANSFER_TILT)
                    if "high" in method
                    else math.degrees(MAX_DOWN_TILT)
                ),
                "resolved_endpoint_branch": resolved_endpoint_used,
                "endpoint_source": selected_endpoint_source,
                "rule": "high corridor; descent only on APP-to-TCP contact leg",
            },
        }

    errors = []
    cross_station_distance = math.dist(stow_position[:2], app_position[:2])

    if robot == "R1":
        try:
            if stem == "SHELL_PICK":
                if math.dist(stow_position, app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R1 edge-grip stow no longer matches pick APP")
                return finish_transit(
                    [list(stow)],
                    app_position[2],
                    "R1 vertical edge-frame pick",
                )
            if stem == "WB1_PLACE":
                scene.set_joints(robot, R1_EDGE_PLACE_APP_JOINTS)
                actual_app = [
                    float(value)
                    for value in sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_app, app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError(
                        f"R1 edge-place APP residual is "
                        f"{math.dist(actual_app, app_position):.3f} m"
                    )
                scene.set_joints(robot, stow)
                transit = validated_joint_line(
                    scene,
                    robot,
                    stow,
                    R1_EDGE_PLACE_APP_JOINTS,
                    transit_exclusions,
                    max_tilt=R1_MAX_DOWN_TILT,
                    min_tip_z=MIN_TRANSIT_Z,
                )
                return finish_transit(
                    transit,
                    app_position[2],
                    "R1 source-overhead to WB1 5deg corridor",
                )
        except RuntimeError as exc:
            errors.append(str(exc))
            scene.set_joints(robot, stow)

    if robot == "R2":
        try:
            if stem == "RAIL_PICK_H":
                if math.dist(stow_position, app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R2 rail carry stow no longer matches pick APP")
                return finish_transit(
                    [list(stow)],
                    app_position[2],
                    "R2 vertical rail-end top pick",
                )
            if stem == "RAIL_PLACE_H":
                scene.set_joints(robot, R2_RAIL_PLACE_APP_JOINTS)
                actual_app = [
                    float(value)
                    for value in sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_app, app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError(
                        f"R2 rail-place APP residual is "
                        f"{math.dist(actual_app, app_position):.3f} m"
                    )
                scene.set_joints(robot, stow)
                transit = validated_joint_line(
                    scene,
                    robot,
                    stow,
                    R2_RAIL_PLACE_APP_JOINTS,
                    transit_exclusions,
                    max_tilt=R2_MAX_DOWN_TILT,
                    min_tip_z=R2_MIN_TRANSIT_Z,
                )
                return finish_transit(
                    transit,
                    app_position[2],
                    "R2 carried-rail 5deg arc corridor",
                )
        except RuntimeError as exc:
            errors.append(str(exc))
            scene.set_joints(robot, stow)

    if robot == "R3" and stem in R3_FIXED_CORRIDORS:
        if stem in {"WB1_PICK", "HANDOFF_PLACE"}:
            assembly_shell = scene.by_alias("Shell_1")
            if stem == "HANDOFF_PLACE":
                transit_exclusions = list(
                    dict.fromkeys(transit_exclusions + [assembly_shell])
                )
            contact_exclusions = list(
                dict.fromkeys(contact_exclusions + [assembly_shell])
            )
        return r3_fixed_down_action(
            scene,
            stem,
            stow,
            app_position,
            tcp_position,
            transit_exclusions,
            contact_exclusions,
        )

    for safe_z in safe_z_candidates:
        # Long transfers use a constrained joint-space planner between two
        # verified high poses.  This avoids incremental IK drift across a
        # metre-scale horizontal span while retaining tilt and height gates.
        if (
            cross_station_distance > 0.25
            and endpoint_app is not None
        ):
            try:
                overhead_position = [app_position[0], app_position[1], safe_z]
                overhead = ik_candidates(
                    scene,
                    robot,
                    overhead_position,
                    endpoint_app,
                    transit_exclusions,
                    attempts=8,
                    max_solutions=1,
                )
                if not overhead:
                    raise RuntimeError(f"no overhead IK at z={safe_z:.3f}")
                try:
                    high_path = validated_joint_line(
                        scene,
                        robot,
                        stow,
                        overhead[0],
                        transit_exclusions,
                        max_tilt=MAX_TRANSFER_TILT,
                        min_tip_z=MIN_TRANSIT_Z,
                        include_other_robots=False,
                    )
                    high_method = "deterministic high joint corridor"
                except RuntimeError as exc:
                    # A joint interpolation can make the TCP dip even when
                    # both endpoints are high.  Fall through to the explicit
                    # Cartesian up-across-down corridor below; do not invoke
                    # a random OMPL fallback that relaxes the down pose.
                    errors.append(str(exc))
                    scene.set_joints(robot, stow)
                else:
                    descent = cartesian_down_line(
                        scene,
                        robot,
                        overhead[0],
                        overhead_position,
                        app_position,
                        transit_exclusions,
                        position_tolerance=TRANSIT_POSITION_TOLERANCE,
                    )
                    return finish_transit(
                        high_path + descent[1:], safe_z, high_method
                    )
            except RuntimeError as exc:
                errors.append(str(exc))
                scene.set_joints(robot, stow)

        midpoint = [
            (stow_position[0] + app_position[0]) / 2.0,
            (stow_position[1] + app_position[1]) / 2.0,
            safe_z,
        ]
        route_candidates: list[tuple[str, list[list[float]]]] = []
        route_candidates.append((
            "Cartesian high corridor",
            [
                [stow_position[0], stow_position[1], safe_z],
                [app_position[0], app_position[1], safe_z],
                app_position,
            ],
        ))
        if cross_station_distance <= 0.25:
            for dx, dy in ((0.16, 0.0), (-0.16, 0.0), (0.0, 0.16), (0.0, -0.16)):
                route_candidates.append((
                    "Cartesian offset high corridor",
                    [
                        [stow_position[0], stow_position[1], safe_z],
                        [midpoint[0] + dx, midpoint[1] + dy, safe_z],
                        [app_position[0], app_position[1], safe_z],
                        app_position,
                    ],
                ))
        for route_method, waypoints in route_candidates:
            try:
                scene.set_joints(robot, stow)
                transit = cartesian_down_route(
                    scene,
                    robot,
                    stow,
                    stow_position,
                    waypoints,
                    transit_exclusions,
                )
                return finish_transit(transit, safe_z, route_method)
            except RuntimeError as exc:
                errors.append(str(exc))
                scene.set_joints(robot, stow)
    detail = " | ".join(errors[-8:]) if errors else "no collision-free down-facing route"
    if robot in {"R6", "R8"}:
        corridor = search_fixed_corridor(
            scene, robot, stem, stow, tcp_position, app_position,
            transit_exclusions, contact_exclusions,
        )
        if corridor is not None:
            print(
                f"[corridor] {robot}_{stem} fixed corridor (transit fallback)",
                flush=True,
            )
            return _fixed_corridor_action(
                scene, robot, stem, stow, corridor,
                app_position, tcp_position,
                transit_exclusions, contact_exclusions,
            )
    raise RuntimeError(f"unable to plan {robot}_{stem}: {detail}")


def build_plan(scene: Scene, scene_path: Path, reuse_checkpoint: bool = True) -> dict:
    if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
        raise RuntimeError("stop the simulation before planning")
    # Planning must not inherit an arbitrary pose left by manual debugging.
    scene.set_all_home()
    checkpoint_path = DEFAULT_PLAN.with_name("eight_arm_cabinet.partial.json")
    checkpoint = {}
    seed_plan: dict = {}
    if DEFAULT_PLAN.is_file():
        try:
            seed_plan = json.loads(DEFAULT_PLAN.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            seed_plan = {}
    if reuse_checkpoint and checkpoint_path.is_file():
        candidate = json.loads(checkpoint_path.read_text(encoding="utf-8"))
        if (
            candidate.get("scene") == fingerprint(scene_path)
            and int(candidate.get("schema_version", 0)) == PLAN_SCHEMA_VERSION
        ):
            checkpoint = candidate

    # Raise each robot's first APP into a collision-free, down-facing stow.
    stows: dict[str, list[float]] = checkpoint.get("stow", {})
    stow_positions: dict[str, list[float]] = checkpoint.get("stow_positions", {})
    for robot in ROBOT_IDS:
        stems = ACTION_TARGETS.get(robot, [])
        if robot not in stows:
            if not stems:
                parked = PARK_ONLY_STOWS[robot]
                parked_joints = [float(value) for value in parked["joints"]]
                parked_position = [float(value) for value in parked["position"]]
                scene.set_joints(robot, parked_joints)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, parked_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError(f"{robot} parked TCP no longer matches the scene")
                validated_joint_line(
                    scene,
                    robot,
                    parked_joints,
                    parked_joints,
                    [],
                    max_tilt=MAX_DOWN_TILT,
                    min_tip_z=MIN_TRANSIT_Z,
                )
                stows[robot] = parked_joints
                stow_positions[robot] = parked_position
                print(f"[stow] {robot} verified conveyor-clear park", flush=True)
                continue
            first = int(scene.sim.getObject(POINTS[f"{robot}_{stems[0]}_APP"]))
            first_app_position = [
                float(v) for v in scene.sim.getObjectPosition(first, -1)
            ]
            if robot == "R1":
                scene.set_joints(robot, R1_EDGE_STOW_JOINTS)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, first_app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R1 edge-grip stow does not match the scene APP")
                validated_joint_line(
                    scene,
                    robot,
                    R1_EDGE_STOW_JOINTS,
                    R1_EDGE_STOW_JOINTS,
                    action_exclusions(scene, robot, stems[0])[0],
                    max_tilt=R1_MAX_DOWN_TILT,
                    min_tip_z=MIN_TRANSIT_Z,
                )
                stows[robot] = list(R1_EDGE_STOW_JOINTS)
                stow_positions[robot] = list(R1_EDGE_STOW_POSITION)
                print(f"[stow] {robot} verified edge-grip carry posture", flush=True)
                continue
            if robot == "R2":
                scene.set_joints(robot, R2_RAIL_STOW_JOINTS)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, first_app_position) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R2 rail carry stow does not match the scene APP")
                validated_joint_line(
                    scene,
                    robot,
                    R2_RAIL_STOW_JOINTS,
                    R2_RAIL_STOW_JOINTS,
                    action_exclusions(scene, robot, stems[0])[0],
                    max_tilt=R2_MAX_DOWN_TILT,
                    min_tip_z=R2_MIN_TRANSIT_Z,
                )
                stows[robot] = list(R2_RAIL_STOW_JOINTS)
                stow_positions[robot] = list(R2_RAIL_STOW_POSITION)
                print(f"[stow] {robot} verified rail-end carry posture", flush=True)
                continue
            if robot == "R3":
                scene.set_joints(robot, R3_RAIL_STOW_JOINTS)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, R3_RAIL_STOW_POSITION) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R3 rail stow does not match the designed position")
                validated_joint_line(
                    scene,
                    robot,
                    R3_RAIL_STOW_JOINTS,
                    R3_RAIL_STOW_JOINTS,
                    action_exclusions(scene, robot, stems[0])[0],
                    max_tilt=R3_MAX_DOWN_TILT,
                    min_tip_z=R3_RAIL_STOW_POSITION[2] - 0.005,
                )
                stows[robot] = list(R3_RAIL_STOW_JOINTS)
                stow_positions[robot] = list(R3_RAIL_STOW_POSITION)
                print(f"[stow] {robot} verified open-rack carry posture", flush=True)
                continue
            if robot == "R4":
                r4_stow = list(R4_TRANSFER_ENDPOINTS["HANDOFF_PICK"]["app"])
                scene.set_joints(robot, r4_stow)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, R4_STOW_POSITION) > IK_POSITION_TOLERANCE:
                    raise RuntimeError("R4 stow does not match the handoff APP")
                validated_joint_line(
                    scene,
                    robot,
                    r4_stow,
                    r4_stow,
                    action_exclusions(scene, robot, stems[0])[0],
                    max_tilt=R4_MAX_DOWN_TILT,
                    min_tip_z=R4_STOW_POSITION[2] - 0.005,
                )
                stows[robot] = r4_stow
                stow_positions[robot] = list(R4_STOW_POSITION)
                print(f"[stow] {robot} verified handoff APP carry posture", flush=True)
                continue
            if robot == "R5":
                scene.set_joints(robot, R5_DEVICE_STOW_JOINTS)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, R5_DEVICE_STOW_POSITION) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R5 device stow does not match the designed position")
                validated_joint_line(
                    scene,
                    robot,
                    R5_DEVICE_STOW_JOINTS,
                    R5_DEVICE_STOW_JOINTS,
                    action_exclusions(scene, robot, stems[0])[0],
                    max_tilt=MAX_DOWN_TILT,
                    min_tip_z=MIN_TRANSIT_Z,
                )
                stows[robot] = list(R5_DEVICE_STOW_JOINTS)
                stow_positions[robot] = list(R5_DEVICE_STOW_POSITION)
                print(f"[stow] {robot} verified east-approach vacuum posture", flush=True)
                continue
            if robot == "R7":
                scene.set_joints(robot, R7_SCREW_STOW_JOINTS)
                actual_position = [
                    float(value)
                    for value in scene.sim.getObjectPosition(scene.tips[robot], -1)
                ]
                if math.dist(actual_position, R7_SCREW_STOW_POSITION) > TRANSIT_POSITION_TOLERANCE:
                    raise RuntimeError("R7 screw stow does not match the first screw APP")
                stows[robot] = list(R7_SCREW_STOW_JOINTS)
                stow_positions[robot] = list(R7_SCREW_STOW_POSITION)
                print(f"[stow] {robot} verified screw-app posture", flush=True)
                continue
            seed = list(HOME)
            if int(seed_plan.get("schema_version", 0)) == 1:
                legacy = seed_plan.get("actions", {}).get(robot, {}).get(stems[0], [])
                if len(legacy) > 1:
                    seed = [float(value) for value in legacy[1]]
            elif robot in seed_plan.get("stow", {}):
                seed = [float(value) for value in seed_plan["stow"][robot]]
            chosen_position: list[float] | None = None
            solutions: list[list[float]] = []
            for position in stow_position_candidates(
                scene, robot, first_app_position
            ):
                solutions = ik_candidates(
                    scene,
                    robot,
                    position,
                    seed,
                    attempts=32,
                    max_solutions=1,
                )
                if solutions:
                    chosen_position = position
                    break
            if not solutions or chosen_position is None:
                raise RuntimeError(f"no collision-free down-facing stow for {robot}")
            stows[robot] = solutions[0]
            stow_positions[robot] = chosen_position
        scene.set_joints(robot, stows[robot])
        print(f"[stow] {robot} collision-free, flange down", flush=True)

    # Zero pose has a horizontal flange and is not a process pose.  Initialize
    # to the verified stows while simulation is stopped so every visible
    # process movement begins and ends with the flange pointing down.
    initial_paths = {robot: [stows[robot]] for robot in ROBOT_IDS}

    for robot in ROBOT_IDS:
        scene.set_joints(robot, stows[robot])
    actions: dict[str, dict[str, dict[str, object]]] = checkpoint.get("actions", {})
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    checkpoint_path.write_text(
        json.dumps(
            {
                "schema_version": PLAN_SCHEMA_VERSION,
                "scene": fingerprint(scene_path),
                "stow": stows,
                "stow_positions": stow_positions,
                "actions": actions,
            },
            ensure_ascii=False,
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    total = sum(len(value) for value in ACTION_TARGETS.values())
    done = 0
    for robot, stems in ACTION_TARGETS.items():
        actions.setdefault(robot, {})
        for stem in stems:
            if stem not in actions[robot]:
                legacy_keyframes = None
                if int(seed_plan.get("schema_version", 0)) == 1:
                    candidate_keyframes = (
                        seed_plan.get("actions", {}).get(robot, {}).get(stem)
                    )
                    if isinstance(candidate_keyframes, list):
                        legacy_keyframes = candidate_keyframes
                elif int(seed_plan.get("schema_version", 0)) == PLAN_SCHEMA_VERSION:
                    endpoint_seeds = (
                        seed_plan.get("actions", {})
                        .get(robot, {})
                        .get(stem, {})
                        .get("endpoint_seeds", {})
                    )
                    if "app" in endpoint_seeds and "tcp" in endpoint_seeds:
                        legacy_keyframes = [
                            [],
                            endpoint_seeds["app"],
                            endpoint_seeds["tcp"],
                        ]
                actions[robot][stem] = plan_action(
                    scene,
                    robot,
                    stem,
                    stows[robot],
                    stow_positions[robot],
                    legacy_keyframes=legacy_keyframes,
                )
                checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
                checkpoint_path.write_text(
                    json.dumps(
                        {
                            "schema_version": PLAN_SCHEMA_VERSION,
                            "scene": fingerprint(scene_path),
                            "stow": stows,
                            "stow_positions": stow_positions,
                            "actions": actions,
                        },
                        ensure_ascii=False,
                        indent=2,
                    ) + "\n",
                    encoding="utf-8",
                )
            done += 1
            print(f"[plan {done:02d}/{total}] {robot}_{stem}", flush=True)
    initial_parts = {}
    for key, (source_alias, _, parent_path) in PARTS.items():
        handle = scene.by_alias(source_alias)
        initial_parts[key] = {
            "matrix": [float(v) for v in scene.sim.getObjectMatrix(handle, -1)],
            "parent": parent_path,
        }
    scene.remove_planner_script()
    return {
        "schema_version": PLAN_SCHEMA_VERSION,
        "scene": {"file": str(scene_path), **fingerprint(scene_path)},
        "orientation": {
            "rule": "physical visible-flange-to-TCP axis points toward world -Z",
            "ik_constraints": "full pose on a per-tool physical-axis TCP marker",
            "maximum_contact_tilt_deg": math.degrees(MAX_DOWN_TILT),
            "maximum_high_transfer_tilt_deg": math.degrees(MAX_TRANSFER_TILT),
        },
        "clearance": {
            "preferred_transit_tcp_z_m": PREFERRED_TRANSIT_Z,
            "minimum_transit_tcp_z_m": MIN_TRANSIT_Z,
            "contact_rule": "only APP-to-TCP legs may descend below the corridor",
        },
        "collision_model": {
            "moving_geometry": "robot tree from joint1, including attached workpiece",
            "environment": "all FiveCR5A_Cell geometry plus the other seven robot trees",
            "cartesian_sample_m": 0.012,
            "joint_sample_deg": math.degrees(MAX_FRAME_DELTA),
        },
        "stow": stows,
        "stow_positions": stow_positions,
        "initial_paths": initial_paths,
        "actions": actions,
        "initial_parts": initial_parts,
    }


def load_or_build_plan(scene: Scene, scene_path: Path, plan_path: Path, rebuild: bool) -> dict:
    current = fingerprint(scene_path)
    if plan_path.is_file() and not rebuild:
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
        expected = {key: plan["scene"][key] for key in ("size", "sha256")}
        if expected == current and int(plan.get("schema_version", 0)) == PLAN_SCHEMA_VERSION:
            print(f"[plan] using {plan_path}")
            return plan
        print("[plan] scene or planner schema changed; rebuilding fixed paths")
    plan = build_plan(scene, scene_path, reuse_checkpoint=not rebuild)
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[plan] wrote {plan_path}")
    return plan


@dataclass
class Track:
    robot: str
    frames: list[list[float]]
    tcp_frame: int
    exclusions: list[int]
    app_frame: int = 0
    carried_part: int | None = None
    carried_mode: str | None = None
    carried_contact_exclusions: list[int] = field(default_factory=list)


class AssemblyRuntime:
    def __init__(self, scene: Scene, plan: dict, speed: float):
        self.scene = scene
        self.sim = scene.sim
        self.plan = plan
        self.speed = speed
        self.assembly: int | None = None
        self.part_handles = {key: scene.by_alias(value[0]) for key, value in PARTS.items()}
        self.ref_handles = {key: scene.by_alias(value[1]) for key, value in PARTS.items()}
        self.pallet = scene.by_alias("Indexing_Pallet_1")
        self.indexing_conveyor = scene.by_alias("Central_Indexing_Conveyor")
        self.pallet_z = float(self.sim.getObjectPosition(self.pallet, -1)[2])
        self.pallet_station = "wb1"
        self.screw_spin = unique_alias(
            self.sim, scene.roots["R7"], "R7T_screw_spin_link"
        )
        self.simulate = True
        self.events: set[str] = set()

    def reset_product(self) -> None:
        try:
            existing = self.scene.by_alias("Assembly_In_Process")
        except RuntimeError:
            existing = -1
        for key, handle in self.part_handles.items():
            parent = int(self.sim.getObject(self.plan["initial_parts"][key]["parent"]))
            self.sim.setObjectParent(handle, parent, True)
            self.sim.setObjectMatrix(handle, -1, self.plan["initial_parts"][key]["matrix"])
        if existing >= 0:
            self.sim.removeObjects([existing])
        self.sim.setObjectPosition(
            self.pallet,
            -1,
            [STATIONS["wb1"][0], STATIONS["wb1"][1], self.pallet_z],
        )
        self.pallet_station = "wb1"
        self.assembly = None

    def set_gripper(self, robot: str, opened: bool) -> None:
        if robot in {"R2", "R5", "R7"}:
            return
        root = self.scene.roots[robot]
        tool = unique_alias(self.sim, root, f"{robot}T")
        left = unique_alias(self.sim, root, f"{robot}T_left_finger_link")
        right = unique_alias(self.sim, root, f"{robot}T_right_finger_link")
        gap = 0.17 if opened else GRIPPER_CLOSED_GAPS.get(robot, 0.035)
        finger_thickness = GRIPPER_FINGER_THICKNESSES.get(robot, 0.020)
        y = (gap + finger_thickness) / 2.0
        lp = list(self.sim.getObjectPosition(left, tool)); lp[1] = y
        rp = list(self.sim.getObjectPosition(right, tool)); rp[1] = -y
        self.sim.setObjectPosition(left, tool, lp)
        self.sim.setObjectPosition(right, tool, rp)

    def snap_part(self, key: str, station: str) -> None:
        if self.assembly is None:
            self.assembly = int(self.sim.createDummy(0.025))
            self.sim.setObjectAlias(self.assembly, "Assembly_In_Process")
            self.sim.setObjectParent(self.assembly, self.pallet, True)
            self.sim.setObjectPosition(self.assembly, -1, STATIONS[station])
            self.sim.setObjectQuaternion(self.assembly, -1, [0.0, 0.0, 0.0, 1.0])
            self.sim.setObjectInt32Param(self.assembly, self.sim.objintparam_visibility_layer, 0)
        part = self.part_handles[key]
        matrix = list(self.sim.getObjectMatrix(self.ref_handles[key], -1))
        for index in range(3):
            matrix[3 + index * 4] += STATIONS[station][index] - REFERENCE_CENTER[index]
        self.sim.setObjectParent(part, self.assembly, True)
        self.sim.setObjectMatrix(part, -1, matrix)

    def index_pallet(self, station: str, *, wait_for: Iterable[str], emits: str) -> None:
        """Move the located assembly to the next locked process station.

        This is deliberately a straight, deterministic conveyor motion.  It
        checks the complete pallet/workpiece tree against every fixture and
        every robot at every frame, while waiving only the intentional
        pallet-to-belt contact.
        """
        required = set(wait_for)
        missing = sorted(required - self.events)
        if missing:
            raise RuntimeError(f"pallet index to {station!r} is waiting for {missing}")
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        if station not in {"wb1", "wb2", "staging", "output"}:
            raise ValueError(f"unknown indexing station: {station}")
        for robot in ROBOT_IDS:
            current = [
                float(self.sim.getJointPosition(handle))
                for handle in self.scene.joints[robot]
            ]
            if config_distance(current, self.plan["stow"][robot]) > math.radians(1.0):
                raise RuntimeError(f"conveyor interlock: {robot} is not at its safe stow")

        self.sim.setObjectParent(self.assembly, self.pallet, True)
        start = [float(value) for value in self.sim.getObjectPosition(self.pallet, -1)]
        end = [float(STATIONS[station][0]), float(STATIONS[station][1]), self.pallet_z]
        moving = int(self.sim.createCollection(0))
        environment = int(self.sim.createCollection(0))
        self.sim.addItemToCollection(moving, self.sim.handle_tree, self.pallet, 0)
        excluded_shapes = set(self.scene.decorative_ground)
        for root in (self.pallet, self.indexing_conveyor):
            excluded_shapes.update(
                int(shape)
                for shape in self.sim.getObjectsInTree(
                    root, self.sim.object_shape_type, 0
                )
            )
        for shape in self.sim.getObjectsInTree(
            self.scene.cell, self.sim.object_shape_type, 0
        ):
            if int(shape) not in excluded_shapes:
                self.sim.addItemToCollection(
                    environment, self.sim.handle_single, int(shape), 0
                )
        for robot in ROBOT_IDS:
            self.scene.add_robot_collision_geometry(environment, robot)
            for shape in self.scene.fixed_base_shapes[robot]:
                self.sim.addItemToCollection(
                    environment, self.sim.handle_single, int(shape), 0
                )
        print(
            f"[run] pallet index {self.pallet_station} -> {station}"
            " | all-robot safe-stow interlock=OK",
            flush=True,
        )
        try:
            steps = max(80, int(math.dist(start[:2], end[:2]) / 0.012))
            for index in range(1, steps + 1):
                blend = quintic(index / steps)
                position = [
                    a + (b - a) * blend for a, b in zip(start, end)
                ]
                self.sim.setObjectPosition(self.pallet, -1, position)
                result, handles = self.sim.checkCollision(moving, environment)
                if int(result) > 0:
                    self.sim.setObjectPosition(self.pallet, -1, start)
                    first = str(self.sim.getObjectAlias(int(handles[0]), 1))
                    second = str(self.sim.getObjectAlias(int(handles[1]), 1))
                    raise RuntimeError(
                        f"pallet collision {first} <-> {second} while indexing "
                        f"{self.pallet_station}->{station} at frame {index}/{steps}"
                    )
                if self.simulate:
                    self.scene.client.step()
        finally:
            self.scene.destroy_collision_pair((moving, environment))
        self.pallet_station = station
        self.events.add(emits)
        print(f"[event] {emits}", flush=True)

    def move_assembly_to(self, station: str) -> None:
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        self.sim.setObjectParent(self.assembly, self.scene.parts_parent, True)
        self.sim.setObjectPosition(self.assembly, -1, STATIONS[station])
        self.sim.setObjectQuaternion(self.assembly, -1, [0.0, 0.0, 0.0, 1.0])

    def attach_part(self, key: str, robot: str) -> None:
        self.set_gripper(robot, False)
        self.sim.setObjectParent(self.part_handles[key], self.scene.tips[robot], True)

    def attach_assembly(self, robot: str) -> None:
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        self.set_gripper(robot, False)
        self.sim.setObjectParent(self.assembly, self.scene.tips[robot], True)

    def track(self, robot: str, stem: str) -> Track:
        action = self.plan["actions"][robot][stem]
        frames = [
            [float(value) for value in frame] for frame in action["frames"]
        ]
        tcp_frame = int(action["tcp_frame"])
        app_seed = [
            float(value) for value in action["endpoint_seeds"]["app"]
        ]
        app_frame = min(
            range(tcp_frame + 1),
            key=lambda index: config_distance(frames[index], app_seed),
        )
        carried_part: int | None = None
        carried_mode: str | None = None
        carried_contact_exclusions: list[int] = []
        key = PICK_PART.get((robot, stem))
        if key is not None:
            carried_part = self.part_handles[key]
            carried_mode = "pick"
            carried_contact_exclusions = [
                int(self.sim.getObject(SOURCE_FIXTURE[key]))
            ]
        else:
            key = PLACE_PART.get((robot, stem))
            if key is not None:
                carried_part = self.part_handles[key]
                carried_mode = "place"
                station = CONTACT_STATION[(robot, stem)]
                carried_contact_exclusions = self.station_fixture_handles(station)
        # Object handles are session-local in CoppeliaSim.  Never trust the
        # handles serialized while the plan was generated; resolve the
        # intentional contact object from the current scene instead.
        _, current_contact_exclusions = action_exclusions(
            self.scene, robot, stem
        )
        if (
            (robot, stem)
            in {
                ("R3", "WB1_PICK"),
                ("R3", "HANDOFF_PLACE"),
                ("R4", "HANDOFF_PICK"),
                ("R4", "WB2_PLACE"),
                ("R6", "WB2_PICK"),
                ("R6", "STAGING_PLACE"),
                ("R8", "STAGING_PICK"),
                ("R8", "OUTPUT_PLACE"),
            }
            and self.assembly is not None
        ):
            current_contact_exclusions.append(self.assembly)
        return Track(
            robot,
            frames,
            tcp_frame,
            list(dict.fromkeys(int(value) for value in current_contact_exclusions)),
            app_frame,
            carried_part,
            carried_mode,
            carried_contact_exclusions,
        )

    def initial_track(self, robot: str) -> Track:
        frames = [
            [float(value) for value in frame]
            for frame in self.plan["initial_paths"][robot]
        ]
        return Track(robot, frames, len(frames) - 1, [])

    def run_tracks(
        self,
        tracks: list[Track],
        callbacks: dict[str, Callable[[], None]] | None = None,
        *,
        simulate: bool = True,
        screw: bool = False,
    ) -> None:
        callbacks = callbacks or {}
        fired: set[str] = set()
        length = max(len(track.frames) for track in tracks)
        collision_sets = []
        all_pairs: list[tuple[int, int]] = []
        for track in tracks:
            moving_exclusions = (
                [track.carried_part] if track.carried_part is not None else []
            )
            robot_pair = self.scene.create_collision_pair(
                track.robot,
                track.exclusions,
                moving_exclusions=moving_exclusions,
            )
            all_pairs.append(robot_pair)
            strict_part_pair: tuple[int, int] | None = None
            contact_part_pair: tuple[int, int] | None = None
            if track.carried_part is not None:
                # Source/destination fixtures may be intentional contacts.
                # R4's complete-cabinet path has now been designed and
                # audited with its own links included, so never waive those
                # collisions.  Other robots retain their legacy waiver until
                # their loaded paths are repaired one by one.
                own_link_waiver = (
                    []
                    if track.robot == "R4"
                    else list(self.scene.collision_shapes[track.robot])
                )
                strict_part_pair = self.scene.create_carried_object_collision_pair(
                    track.robot,
                    track.carried_part,
                    list(track.carried_contact_exclusions)
                    + own_link_waiver,
                )
                contact_part_pair = self.scene.create_carried_object_collision_pair(
                    track.robot,
                    track.carried_part,
                    list(track.carried_contact_exclusions)
                    + own_link_waiver,
                )
                all_pairs.extend([strict_part_pair, contact_part_pair])
            collision_sets.append((track, robot_pair, strict_part_pair, contact_part_pair))
        try:
            for frame_index in range(length):
                positions = {
                    track.robot: track.frames[min(frame_index, len(track.frames) - 1)]
                    for track in tracks
                }
                spin = (self.screw_spin, frame_index * 0.45) if screw else None
                active_pairs: list[tuple[int, int]] = []
                for track, robot_pair, strict_part_pair, contact_part_pair in collision_sets:
                    active_pairs.append(robot_pair)
                    if strict_part_pair is None or contact_part_pair is None:
                        continue
                    outbound_app = 2 * track.tcp_frame - track.app_frame
                    if track.carried_mode == "pick":
                        if frame_index < track.tcp_frame:
                            continue
                        if frame_index <= outbound_app:
                            active_pairs.append(contact_part_pair)
                        else:
                            active_pairs.append(strict_part_pair)
                    elif track.carried_mode == "place":
                        if frame_index < track.app_frame:
                            active_pairs.append(strict_part_pair)
                        elif frame_index <= track.tcp_frame:
                            active_pairs.append(contact_part_pair)
                try:
                    self.scene.apply_frame(positions, active_pairs, spin)
                except RuntimeError as exc:
                    # Freeze at the last known safe frame.  The outer handler
                    # subsequently stops the simulation and removes bridges.
                    if int(self.sim.getSimulationState()) != int(self.sim.simulation_stopped):
                        self.sim.pauseSimulation()
                    raise RuntimeError(
                        f"{exc} at coordinated frame {frame_index}/{length - 1}"
                    ) from exc
                for track in tracks:
                    if frame_index >= track.tcp_frame and track.robot not in fired:
                        callback = callbacks.get(track.robot)
                        if callback is not None:
                            callback()
                        fired.add(track.robot)
                if simulate:
                    self.scene.client.step()
        finally:
            for pair in all_pairs:
                self.scene.destroy_collision_pair(pair)

    def move_to_stows(self, simulate: bool) -> None:
        for robot in ROBOT_IDS:
            self.scene.set_joints(robot, self.plan["stow"][robot])

    def preflight(self) -> None:
        print(
            "[preflight] replaying the complete process with fixtures and attached workpieces",
            flush=True,
        )
        self.reset_product()
        self.scene.set_all_home()
        self.simulate = False
        self.events.clear()
        self.move_to_stows(simulate=False)
        run_process(self)
        self.reset_product()
        self.scene.set_all_home()
        self.simulate = True
        self.events.clear()
        print("[preflight] full geometry and carried-workpiece sweep passed", flush=True)

    def pick(self, robot: str, stem: str, key: str) -> tuple[Track, Callable[[], None]]:
        self.set_gripper(robot, True)
        return self.track(robot, stem), lambda: self.attach_part(key, robot)

    def place(self, robot: str, stem: str, key: str, station: str) -> tuple[Track, Callable[[], None]]:
        def release() -> None:
            self.snap_part(key, station)
            # R3 retracts its short rail fingers while still closed.  Opening
            # inside the cabinet would sweep the jaws through the side wall;
            # the following pick/transfer action opens them after the arm has
            # returned to its clear APP/stow posture.
            if not (robot == "R3" and stem in {"RAIL_PLACE_A", "RAIL_PLACE_B"}):
                self.set_gripper(robot, True)
        track = self.track(robot, stem)
        if self.assembly is not None:
            # Only the carried part may contact the product on APP -> TCP.
            # The robot links and tool still see the complete assembly as an
            # obstacle throughout the track.
            track.carried_contact_exclusions.append(self.assembly)
        track.carried_contact_exclusions = list(
            dict.fromkeys(track.carried_contact_exclusions)
        )
        return track, release

    def transfer_pick(self, robot: str, stem: str) -> tuple[Track, Callable[[], None]]:
        self.set_gripper(robot, True)
        track = self.track(robot, stem)
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        track.exclusions.append(self.assembly)
        source_station = {
            ("R3", "WB1_PICK"): "wb1",
            ("R4", "HANDOFF_PICK"): "handoff",
            ("R6", "WB2_PICK"): "wb2",
            ("R8", "STAGING_PICK"): "staging",
        }[(robot, stem)]
        track.carried_part = self.assembly
        track.carried_mode = "pick"
        track.carried_contact_exclusions = self.station_fixture_handles(
            source_station
        )
        return track, lambda: self.attach_assembly(robot)

    def transfer_place(self, robot: str, stem: str, station: str) -> tuple[Track, Callable[[], None]]:
        def release() -> None:
            self.move_assembly_to(station)
            self.set_gripper(robot, True)
        track = self.track(robot, stem)
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        track.exclusions.append(self.assembly)
        track.carried_part = self.assembly
        track.carried_mode = "place"
        track.carried_contact_exclusions = self.station_fixture_handles(station)
        return track, release

    def station_fixture_handles(self, station: str) -> list[int]:
        return [
            int(self.sim.getObject(path))
            for path in STATION_FIXTURE_PATHS[station]
        ]

    def execute_pair(
        self,
        entries: list[tuple[Track, Callable[[], None]]],
        label: str,
        *,
        wait_for: Iterable[str] = (),
        emits: Iterable[str] = (),
    ) -> None:
        required = set(wait_for)
        missing = sorted(required - self.events)
        if missing:
            raise RuntimeError(f"stage {label!r} is waiting for events: {missing}")
        print(
            f"[run] {label} | active={','.join(item[0].robot for item in entries)}"
            f" | after={','.join(sorted(required)) or '-'}",
            flush=True,
        )
        self.run_tracks(
            [item[0] for item in entries],
            {item[0].robot: item[1] for item in entries},
            simulate=self.simulate,
        )
        produced = set(emits)
        self.events.update(produced)
        if produced:
            print(f"[event] {', '.join(sorted(produced))}", flush=True)

    def execute_screw(
        self,
        track: Track,
        index: int,
        *,
        wait_for: Iterable[str],
        emits: str,
    ) -> None:
        required = set(wait_for)
        missing = sorted(required - self.events)
        if missing:
            raise RuntimeError(f"screw {index} is waiting for events: {missing}")
        print(f"[run] R7 screw {index}/4 | after={','.join(sorted(required))}", flush=True)
        self.run_tracks([track], screw=True, simulate=self.simulate)
        self.events.add(emits)
        print(f"[event] {emits}", flush=True)

    def staging_slide(self) -> None:
        """Animate the staging conveyor carrying the cabinet to the screw
        stop."""
        self._slide_to("staging conveyor slide -> screw stop", 0.15, 0.27)

    def staging_slide_final(self) -> None:
        """Carry the cabinet from the screw stop to the R8 pick point."""
        self._slide_to("staging conveyor slide -> R8 pick point", 0.30, 0.10)

    def _slide_to(self, label: str, x: float, y: float) -> None:
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        print(f"[run] {label}", flush=True)
        start = list(self.sim.getObjectPosition(self.assembly, -1))
        end = [x, y, start[2]]
        for index in range(1, 61):
            blend = quintic(index / 60.0)
            self.sim.setObjectPosition(
                self.assembly, -1,
                [a + (b - a) * blend for a, b in zip(start, end)],
            )
            if self.simulate:
                self.scene.client.step()

    def handoff_slide(self) -> None:
        """Animate the handoff conveyor carrying the cabinet west -> east."""
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        print("[run] handoff conveyor slide west -> east", flush=True)
        start = list(self.sim.getObjectPosition(self.assembly, -1))
        end = [float(value) for value in R4_HANDOFF_CENTER]
        for index in range(1, 61):
            blend = quintic(index / 60.0)
            self.sim.setObjectPosition(
                self.assembly, -1,
                [a + (b - a) * blend for a, b in zip(start, end)],
            )
            if self.simulate:
                self.scene.client.step()

    def conveyor_to_bin(self) -> None:
        if self.assembly is None:
            raise RuntimeError("assembly root does not exist")
        print("[run] finished conveyor -> Finished_Bin", flush=True)
        start = list(self.sim.getObjectPosition(self.assembly, -1))
        end = [2.05, -0.92, 0.20]
        for index in range(1, 101):
            blend = quintic(index / 100.0)
            self.sim.setObjectPosition(
                self.assembly, -1,
                [a + (b - a) * blend for a, b in zip(start, end)],
            )
            if self.simulate:
                self.scene.client.step()


def process_stages() -> list[list[tuple[str, str]]]:
    """Robot-motion groups; conveyor index steps occur between these groups."""
    return [
        [("R1", "SHELL_PICK"), ("R2", "RAIL_PICK_H"), ("R3", "RAIL_PICK_A")],
        [("R1", "WB1_PLACE")], [("R2", "RAIL_PLACE_H")],
        [("R3", "RAIL_PLACE_A")], [("R3", "RAIL_PICK_B")],
        [("R3", "RAIL_PLACE_B")],
        [("R5", "PLC_PICK"), ("R6", "SERVO_PICK")],
        [("R5", "PLC_PLACE")],
        [("R6", "SERVO_PLACE"), ("R5", "PSU_PICK")],
        [("R5", "PSU_PLACE"), ("R6", "DMA_PICK")],
        [("R6", "DMA_PLACE")],
        [("R6", "CONTACTOR_PICK")], [("R6", "CONTACTOR_PLACE")],
        [("R6", "BREAKER_PICK")], [("R6", "BREAKER_PLACE")],
        [("R7", "SCREW_1")], [("R7", "SCREW_2")],
        [("R7", "SCREW_3")], [("R7", "SCREW_4")],
    ]


def run_process(runtime: AssemblyRuntime) -> None:
    p1 = runtime.pick("R1", "SHELL_PICK", "shell")
    p2 = runtime.pick("R2", "RAIL_PICK_H", "rail_h")
    p3 = runtime.pick("R3", "RAIL_PICK_A", "rail_a")
    runtime.execute_pair(
        [p1, p2, p3],
        "R1/R2/R3 parallel material pick",
        emits=("WB1_MATERIALS_HELD",),
    )
    runtime.execute_pair(
        [runtime.place("R1", "WB1_PLACE", "shell", "wb1")],
        "R1 shell -> WB1",
        wait_for=("WB1_MATERIALS_HELD",),
        emits=("SHELL_AT_WB1",),
    )
    runtime.execute_pair(
        [runtime.place("R2", "RAIL_PLACE_H", "rail_h", "wb1")],
        "R2 horizontal rail install",
        wait_for=("SHELL_AT_WB1",),
        emits=("RAIL_H_DONE",),
    )
    runtime.execute_pair(
        [runtime.place("R3", "RAIL_PLACE_A", "rail_a", "wb1")],
        "R3 vertical rail A install",
        wait_for=("RAIL_H_DONE",),
        emits=("RAIL_A_DONE",),
    )
    runtime.execute_pair(
        [runtime.pick("R3", "RAIL_PICK_B", "rail_b")],
        "R3 vertical rail B pick",
        wait_for=("RAIL_A_DONE",),
        emits=("RAIL_B_HELD",),
    )
    runtime.execute_pair(
        [runtime.place("R3", "RAIL_PLACE_B", "rail_b", "wb1")],
        "R3 vertical rail B install",
        wait_for=("RAIL_B_HELD",),
        emits=("WB1_ASSEMBLY_DONE",),
    )

    runtime.index_pallet(
        "wb2",
        wait_for=("WB1_ASSEMBLY_DONE",),
        emits="WB2_READY",
    )
    entries = [
        runtime.pick("R5", "PLC_PICK", "plc"),
        runtime.pick("R6", "SERVO_PICK", "servo"),
    ]
    runtime.execute_pair(
        entries,
        "R5/R6 parallel device prefetch",
        wait_for=("WB2_READY",),
        emits=("WB2_FIRST_DEVICES_HELD",),
    )
    runtime.execute_pair(
        [runtime.place("R5", "PLC_PLACE", "plc", "wb2")],
        "R5 PLC install",
        wait_for=("WB2_READY", "WB2_FIRST_DEVICES_HELD"),
        emits=("PLC_DONE",),
    )
    runtime.execute_pair(
        [
            runtime.place("R6", "SERVO_PLACE", "servo", "wb2"),
            runtime.pick("R5", "PSU_PICK", "psu"),
        ],
        "R6 servo install + R5 PSU prefetch",
        wait_for=("PLC_DONE",),
        emits=("SERVO_DONE", "PSU_HELD"),
    )
    runtime.execute_pair(
        [
            runtime.place("R5", "PSU_PLACE", "psu", "wb2"),
            runtime.pick("R6", "DMA_PICK", "dma"),
        ],
        "R5 PSU install + R6 DMA prefetch",
        wait_for=("SERVO_DONE", "PSU_HELD"),
        emits=("PSU_DONE", "DMA_HELD"),
    )
    runtime.execute_pair(
        [runtime.place("R6", "DMA_PLACE", "dma", "wb2")],
        "R6 DMA install",
        wait_for=("PSU_DONE", "DMA_HELD"),
        emits=("DMA_DONE",),
    )
    previous_event = "DMA_DONE"
    for stem, key, label in (
        ("CONTACTOR", "contactor", "contactor"),
        ("BREAKER", "breaker", "breaker"),
    ):
        held_event = f"{stem}_HELD"
        done_event = f"{stem}_DONE"
        runtime.execute_pair(
            [runtime.pick("R6", f"{stem}_PICK", key)],
            f"R6 {label} pick",
            wait_for=(previous_event,),
            emits=(held_event,),
        )
        runtime.execute_pair(
            [runtime.place("R6", f"{stem}_PLACE", key, "wb2")],
            f"R6 {label} install",
            wait_for=(held_event,),
            emits=(done_event,),
        )
        previous_event = done_event
    runtime.index_pallet(
        "staging",
        wait_for=(previous_event,),
        emits="STAGING_READY",
    )

    previous_event = "STAGING_READY"
    for index in range(1, 5):
        screw_track = runtime.track("R7", f"SCREW_{index}")
        if runtime.assembly is not None:
            # Contact between screwdriver bit and cabinet is intentional; the
            # rest of the R7 moving chain remains checked against all fixtures.
            screw_track.exclusions.append(runtime.assembly)
        screw_event = f"SCREW_{index}_DONE"
        runtime.execute_screw(
            screw_track,
            index,
            wait_for=(previous_event,),
            emits=screw_event,
        )
        previous_event = screw_event
    runtime.index_pallet(
        "output",
        wait_for=("SCREW_4_DONE",),
        emits="OUTPUT_READY",
    )
    runtime.events.add("CYCLE_COMPLETE")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--scene", type=Path, default=SCENE_FILE)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--rebuild-plan", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0, help="0.2..3.0; larger is faster")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scene_path = args.scene.expanduser().resolve()
    plan_path = args.plan.expanduser().resolve()
    client = RemoteAPIClient(args.host, args.port)
    client.timeout = 60.0
    try:
        import zmq

        client.socket.setsockopt(zmq.RCVTIMEO, 60000)
        client.socket.setsockopt(zmq.SNDTIMEO, 60000)
        client.socket.setsockopt(zmq.LINGER, 0)
    except (ImportError, AttributeError):
        pass
    scene = Scene(client)
    live_path = Path(scene.sim.getStringParam(scene.sim.stringparam_scene_path_and_name)).resolve()
    if live_path != scene_path:
        raise RuntimeError(f"open scene is {live_path}, expected {scene_path}")
    if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
        raise RuntimeError("the simulation must be stopped before this controller starts")

    try:
        plan = load_or_build_plan(scene, scene_path, plan_path, args.rebuild_plan)
    finally:
        scene.remove_planner_script()
    if args.plan_only:
        return 0
    runtime = AssemblyRuntime(scene, plan, args.speed)
    runtime.reset_product()
    scene.set_all_home()
    scene.install_batch_script()
    try:
        if not args.skip_preflight:
            runtime.preflight()
        runtime.move_to_stows(simulate=False)
        client.setStepping(True)
        scene.sim.startSimulation()
        client.step()
        run_process(runtime)
        scene.sim.pauseSimulation()
        client.setStepping(False)
        scene.remove_batch_script()
        print("[done] cabinet assembly complete; simulation is paused with the product in Finished_Bin")
        return 0
    except Exception:
        try:
            if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
                scene.sim.stopSimulation()
        finally:
            client.setStepping(False)
            scene.remove_batch_script()
        raise


if __name__ == "__main__":
    raise SystemExit(main())
