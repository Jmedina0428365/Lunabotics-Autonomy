"""Lunabotics 2026 Arena — exact geometry from the new (2025-2026) guidebook.

Coordinate frame matches Rover/UMRS/src/main.rs and Rover/Tools/sim.py:
  * x : along the 8.10 m side   (0 .. 8.10)
  * z : along the 4.57 m side   (0 .. 4.57)
  * The berm / digging zone is at +x, the starting zone at -x.

Key guidebook facts encoded here:
  * Arena: 26.5 ft x 15 ft = 8.0772 m x 4.572 m  (team uses 8.10 x 4.57)
  * Berm: 5 ft x 3 ft x ~1 ft  -> center 6.80 m from the start wall,
    centered at z = 2.57 m (main.rs TARGET_BERM_X/Z).
  * Dig target inside the berm: (1.00, 1.00) relative to berm corner
    (main.rs TARGET_DIG_X/Z).
  * ArUco marker mounted on the far (+x) wall at (8.10, 3.50)
    (Rover/Tools/sim.py ARUCO_X/ARUCO_Z).
  * Obstacles: rocks/boulders 30-40 cm diameter, craters 40-50 cm wide.
"""

import math
import random

# ------------------------------------------------------------------
# Arena constants (meters) — new guidebook
# ------------------------------------------------------------------
ARENA_L = 8.10          # 26.5 ft side (start wall -> berm wall)
ARENA_W = 4.57          # 15 ft side

ZONE_BOUNDARY_X = ARENA_L - 4.0   # 4.00 m: all but last 4 m is "starting zone"

# Berm (regolith pile) — 5 ft x 3 ft footprint at the far end
BERM_CX = 6.80
BERM_CZ = 2.57
BERM_W = 1.52           # 5 ft along x
BERM_H = 0.91           # 3 ft along z
BERM_X0 = BERM_CX - BERM_W / 2.0
BERM_X1 = BERM_CX + BERM_W / 2.0
BERM_Z0 = BERM_CZ - BERM_H / 2.0
BERM_Z1 = BERM_CZ + BERM_H / 2.0

# ArUco marker on the far wall (rear camera localization reference)
ARUCO_X = ARENA_L
ARUCO_Z = 3.50

# Guidebook obstacle size limits
ROCK_DIAM_MIN, ROCK_DIAM_MAX = 0.30, 0.40        # boulders: 30-40 cm dia
CRATER_DIAM_MIN, CRATER_DIAM_MAX = 0.40, 0.50    # craters: 40-50 cm wide

# UMRS main.rs planning bounds (rover must stay inside these)
BOUNDS_X_MIN = -0.20
BOUNDS_X_MAX = 7.80
BOUNDS_Z_MIN = -0.20
BOUNDS_Z_MAX = 3.57


class Arena:
    """Mutable arena state: craters, boulders, rover spawn."""

    def __init__(self):
        self.craters = [
            {"x": 5.0, "z": 3.4, "r": 0.25},
            {"x": 3.0, "z": 3.4, "r": 0.20},
        ]
        self.boulders = [
            {"x": 5.0, "z": 1.2, "r": 0.15},
            {"x": 3.5, "z": 1.2, "r": 0.20},
        ]
        self.rover = {"x": 7.0, "z": 3.5, "yaw": -math.pi / 2}

    # ---------------- deterministic defaults ----------------
    def reset_default(self):
        self.__init__()

    # ---------------- randomization ----------------
    def randomize(self, seed=None, n_craters=4, n_boulders=4):
        rng = random.Random(seed)
        self.craters = []
        self.boulders = []
        for _ in range(n_craters):
            r = rng.uniform(CRATER_DIAM_MIN, CRATER_DIAM_MAX) / 2.0
            self.craters.append({
                "x": rng.uniform(1.0 + r, 5.9 - r),
                "z": rng.uniform(0.4 + r, ARENA_W - 0.4 - r),
                "r": round(r, 4),
            })
        for _ in range(n_boulders):
            r = rng.uniform(ROCK_DIAM_MIN, ROCK_DIAM_MAX) / 2.0
            self.boulders.append({
                "x": rng.uniform(1.0 + r, 5.9 - r),
                "z": rng.uniform(0.4 + r, ARENA_W - 0.4 - r),
                "r": round(r, 4),
            })
        return self

    # ---------------- serialization ----------------
    def to_dict(self):
        return {
            "arena": {"length_m": ARENA_L, "width_m": ARENA_W},
            "berm": {"cx": BERM_CX, "cz": BERM_CZ, "w": BERM_W, "h": BERM_H},
            "aruco": {"x": ARUCO_X, "z": ARUCO_Z},
            "craters": [dict(c) for c in self.craters],
            "boulders": [dict(b) for b in self.boulders],
            "rover_spawn": dict(self.rover),
        }

    def load_dict(self, d):
        self.craters = [dict(c) for c in d.get("craters", [])]
        self.boulders = [dict(b) for b in d.get("boulders", [])]
        if "rover_spawn" in d:
            self.rover = dict(d["rover_spawn"])
        return self
