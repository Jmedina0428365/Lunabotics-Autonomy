"""Deterministic scripted-input simulation engine.

This is the heart of the sim used for testing: the central coordinator
(Rover/UMRS/src/main.rs) and the planner (Rover/UMRS/src/planner.rs) are
NOT modified and NOT required — their decision output is replaced by a
hard-coded script so we can verify the simulation itself (kinematics,
synthetic sensors, simulated CAN bus) produces accurate, reproducible
results.

The script mirrors the manual-control key mapping in main.rs:
    w -> m1=m2=-1, m3=m4=+1   (forward)
    s -> m1=m2=+1, m3=m4=-1   (backward)
    a -> m1..m4 = +1          (turn left / pivot)
    d -> m1..m4 = -1          (turn right / pivot)
plus excavation actions:
    q -> excavate (linear actuator down to EXCAVATE_TARGET_DEPTH_CM,
                   belt scoops EXCAVATE_BELT_TOTAL_CM)
    e -> deposit  (deposition belt travels DUMP_BELT_TRAVEL_CM out)
"""

import math

from .arena import Arena
from .can_sim import SimCanBus, CanFrame, SABERTOOTH_CMD_ID, velocity_cmd_id
from .sensors import SyntheticSensors, CAL_M, CAL_B

# Constants mirrored from Rover/UMRS/src/main.rs (read-only reference)
BASE_SPEED_RPM = 150.0
EXCAVATE_SPEED_RPM = 300.0
EXCAVATE_TARGET_DEPTH_CM = 12.0
EXCAVATE_BELT_TOTAL_CM = -25.0
DUMP_BELT_TRAVEL_CM = -43.0
KP_ACTUATOR = 0.8
ACT_MIN_SPEED = 12
DEADBAND_ADC = 5
KP_DEPOSITION = 2.8
DEPO_MIN_SPEED = 20
DEADBAND_DEPO_CM = 0.2

# Hard-coded test script: (action, duration_seconds)
SCRIPT = [
    ("w", 2.0),      # drive forward
    ("a", 1.0),      # pivot left
    ("w", 1.0),      # drive forward
    ("d", 1.0),      # pivot right
    ("w", 1.5),      # drive forward
    ("q", 3.0),      # excavate
    ("s", 1.0),      # reverse
    ("e", 3.0),      # deposit
]


class SimEngine:
    def __init__(self, arena=None, bus=None):
        self.arena = arena or Arena()
        self.bus = bus or SimCanBus()
        self.sensors = SyntheticSensors(self.arena, self.bus)
        self.sim_time = 0.0
        self.script_index = 0
        self.script_elapsed = 0.0
        self.history = []           # snapshots per recorded step
        self.excavating = False
        self.depositing = False
        self.action_timer = 0.0

    # ------------------------------------------------------------------
    def reset(self):
        self.arena.reset_default()
        self.bus = SimCanBus()
        self.sensors = SyntheticSensors(self.arena, self.bus)
        self.sim_time = 0.0
        self.script_index = 0
        self.script_elapsed = 0.0
        self.history = []
        self.excavating = False
        self.depositing = False
        self.action_timer = 0.0

    def current_action(self):
        if self.script_index < len(SCRIPT):
            return SCRIPT[self.script_index]      # (key, duration)
        return None

    # ------------------------------------------------------------------
    def _apply_manual_key(self, key):
        """Translate a hard-coded key into motor commands exactly like
        the Manual state block in main.rs."""
        if key == "w":
            cmds = (-1.0, -1.0, 1.0, 1.0)
        elif key == "s":
            cmds = (1.0, 1.0, -1.0, -1.0)
        elif key == "a":
            cmds = (1.0, 1.0, 1.0, 1.0)
        elif key == "d":
            cmds = (-1.0, -1.0, -1.0, -1.0)
        else:
            cmds = (0.0, 0.0, 0.0, 0.0)
        for dev, c in zip((1, 2, 3, 4), cmds):
            rpm = BASE_SPEED_RPM * c
            self.bus.send_frame(CanFrame(
                velocity_cmd_id(dev),
                struct_pack_f32(rpm) + b"\x00\x00\x00\x01"))

    # ------------------------------------------------------------------
    def _apply_actuators(self, dt):
        """P-position control for the linear actuators, mirroring the
        control law in main.rs (ADC/depo cm P-controllers)."""
        s = self.sensors
        act_speed = depo_speed = 0

        if self.excavating:
            target_adc = int(EXCAVATE_TARGET_DEPTH_CM * CAL_M + CAL_B)
            err = target_adc - s.act_adc
            if abs(err) >= DEADBAND_ADC:
                v = int(err * KP_ACTUATOR)
                v += ACT_MIN_SPEED if v > 0 else -ACT_MIN_SPEED
                act_speed = max(-127, min(127, int(v * 0.3)))
            elif self.action_timer > 1.0 and s.depo_cm > EXCAVATE_BELT_TOTAL_CM:
                depo_speed = -60     # belt scoop-in
        if self.depositing:
            target = EXCAVATE_BELT_TOTAL_CM + DUMP_BELT_TRAVEL_CM
            err = target - s.depo_cm
            if abs(err) >= DEADBAND_DEPO_CM:
                v = int(err * KP_DEPOSITION)
                v += DEPO_MIN_SPEED if v > 0 else -DEPO_MIN_SPEED
                depo_speed = max(-127, min(127, v))

        self.bus.send_frame(CanFrame(
            SABERTOOTH_CMD_ID,
            act_speed.to_bytes(2, "little", signed=True)
            + depo_speed.to_bytes(2, "little", signed=True) + b"\x00\x00\x00\x00"))

    # ------------------------------------------------------------------
    def step(self, dt: float, record=False, manual_key=None):
        """Advance the sim one tick.

        If manual_key (w/a/s/d/q/e) is given it overrides the script
        (used by the GUI's live manual-control mode); otherwise the
        hard-coded deterministic SCRIPT drives the rover.
        """
        action = self.current_action() if manual_key is None else None

        if manual_key is not None:
            key = manual_key
        elif action is None:
            key = None
        else:
            key = action[0]

        if key in ("w", "a", "s", "d"):
            self._apply_manual_key(key)
        else:
            self._apply_manual_key(None)   # stop wheels

        if key == "q" and not self.excavating:
            self.excavating, self.action_timer = True, 0.0
        if key == "e" and not self.depositing:
            self.depositing, self.action_timer = True, 0.0

        if self.excavating or self.depositing:
            self.action_timer += dt
            dur = action[1] if action is not None else 3.0
            if self.action_timer >= dur:
                self.excavating = self.depositing = False

        self._apply_actuators(dt)
        self.sensors.consume_can_commands()
        self.sensors.step(dt)
        self.sim_time += dt

        if manual_key is None and action is not None:
            self.script_elapsed += dt
            if self.script_elapsed >= action[1]:
                self.script_index += 1
                self.script_elapsed = 0.0

        if record:
            self.record()

    # ------------------------------------------------------------------
    def record(self):
        snap = self.sensors.snapshot()
        snap["t"] = round(self.sim_time, 3)
        snap["script_action"] = self.current_action()
        snap["can_frames_tx"] = len(self.bus.tx_log)
        self.history.append(snap)

    def run_script(self, dt=1.0 / 60.0):
        """Run the full hard-coded script deterministically, recording."""
        while self.current_action() is not None:
            self.step(dt, record=True)
        return self.history

    # ------------------------------------------------------------------
    def export_json(self):
        return {
            "sim_version": "1.0",
            "mode": "deterministic-hardcoded-script",
            "note": ("Coordinator (main.rs) and planner (planner.rs) are "
                     "unmodified; decisions here are hard-coded inputs used "
                     "to validate the simulation."),
            "guidebook_arena": self.arena.to_dict(),
            "script": [{"key": k, "duration_s": d} for k, d in SCRIPT],
            "final_rover_state": self.sensors.snapshot(),
            "telemetry_payload_final": self.sensors.telemetry_payload(),
            "history": self.history,
        }


def struct_pack_f32(v: float) -> bytes:
    import struct
    return struct.pack("<f", v)
