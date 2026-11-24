"""Synthetic sensor feeds: wheel encoders, IMU, linear actuators (ADC +
deposition linear encoder), cameras (front 16-band depth scanner and rear
ArUco camera). Mirrors the message formats of Rover/Drivers/perception.py
and the CAN telemetry parsed by Rover/UMRS/src/main.rs.

The physics constants match main.rs:
  WHEEL_RADIUS_M = 0.13335, TRACK_WIDTH_M = 1.06625, GEAR_RATIO = 64
  CAL_M = 42.73, CAL_B = 20.0   (depth ADC = cm * CAL_M + CAL_B)
"""

import math

from .arena import Arena, ARENA_W, ARUCO_X, ARUCO_Z
from .can_sim import SimCanBus

WHEEL_RADIUS_M = 0.13335
TRACK_WIDTH_M = 1.06625
GEAR_RATIO = 64.0
CAL_M = 42.73
CAL_B = 20.0

# Camera models (same values as Rover/Tools/sim.py / perception.py)
FRONT_FOV_DEG = 80.0
FRONT_MIN_DIST = 0.3
FRONT_MAX_DIST = 3.0
NUM_BANDS = 16
REAR_FOV_DEG = 20.0
REAR_MIN_DIST = 0.5
REAR_MAX_DIST = 6.5

MAX_WHEEL_RPM = 600.0            # output-shaft command limit used by sim
DEPO_SPEED_MAX_CM_S = 12.0       # belt speed at full i16 duty
ACT_ADC_RATE = 60.0              # depth travel rate at full duty


def ray_circle_intersect(rx, rz, dx, dz, cx, cz, radius):
    fx, fz = rx - cx, rz - cz
    a = dx * dx + dz * dz
    b = 2.0 * (fx * dx + fz * dz)
    c = fx * fx + fz * fz - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    if t1 > 0:
        return t1
    if t2 > 0:
        return t2
    return None


def ray_segment_intersect(rx, rz, dx, dz, x1, z1, x2, z2):
    sx, sz = x2 - x1, z2 - z1
    denom = dx * sz - dz * sx
    if abs(denom) < 1e-8:
        return None
    t = ((x1 - rx) * sz - (z1 - rz) * sx) / denom
    u = ((x1 - rx) * dz - (z1 - rz) * dx) / denom
    return t if (t > 0 and 0.0 <= u <= 1.0) else None


class SyntheticSensors:
    """Owns the simulated rover physical state and feeds synthetic data
    into the simulated CAN bus + ZMQ perception stream."""

    def __init__(self, arena: Arena, bus: SimCanBus):
        self.arena = arena
        self.bus = bus
        r = arena.rover
        self.x, self.z, self.yaw = r["x"], r["z"], r["yaw"]
        self.target_rpm = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
        self.actual_rpm = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
        self.act_adc = int(CAL_B)          # bucket fully retracted
        self.depo_cm = 0.0                 # deposition belt position
        self.sabertooth = (0, 0)           # (act_speed, depo_speed)
        self.odometry = {"left": 0.0, "right": 0.0}
        self.load_kg = 0.0

    # ------------------------------------------------------------------
    def consume_can_commands(self):
        """Drain frames the coordinator sent onto the bus."""
        while True:
            frame = self.bus.tx_log.pop(0) if self.bus.tx_log else None
            if frame is None:
                break
            v = SimCanBus.parse_velocity_cmd(frame)
            if v is not None:
                dev, rpm = v
                if dev in self.target_rpm:
                    self.target_rpm[dev] = max(-MAX_WHEEL_RPM,
                                               min(MAX_WHEEL_RPM, rpm))
                continue
            s = SimCanBus.parse_sabertooth_cmd(frame)
            if s is not None:
                self.sabertooth = s

    # ------------------------------------------------------------------
    def step(self, dt: float):
        """Advance rover kinematics + actuators, then publish telemetry."""
        # ideal wheel speeds -> simple first-order motor response
        for dev in (1, 2, 3, 4, 5):
            self.actual_rpm[dev] += (self.target_rpm[dev] - self.actual_rpm[dev]) \
                * min(1.0, dt * 10.0)

        # Track drive: main.rs treats m1/m2 as left, m3/m4 as right and
        # negates left; negative cmd == forward.
        v_left = -((self.actual_rpm[1] + self.actual_rpm[2]) / 2.0) / GEAR_RATIO \
            * (2.0 * math.pi * WHEEL_RADIUS_M) / 60.0
        v_right = ((self.actual_rpm[3] + self.actual_rpm[4]) / 2.0) / GEAR_RATIO \
            * (2.0 * math.pi * WHEEL_RADIUS_M) / 60.0

        v_fwd = (v_left + v_right) / 2.0
        omega = (v_left - v_right) / TRACK_WIDTH_M

        self.x += v_fwd * math.sin(self.yaw) * dt
        self.z += v_fwd * math.cos(self.yaw) * dt
        self.yaw = (self.yaw + omega * dt + math.pi) % (2 * math.pi) - math.pi
        self.odometry["left"] += v_left * dt
        self.odometry["right"] += v_right * dt

        # Actuator dynamics (linear actuator depth + deposition belt)
        act_speed, depo_speed = self.sabertooth
        self.act_adc = max(int(CAL_B),
                           min(int(CAL_B + 30.0 * CAL_M),
                               self.act_adc + act_speed * ACT_ADC_RATE * dt))
        self.depo_cm += (depo_speed / 127.0) * DEPO_SPEED_MAX_CM_S * dt

        self._publish_telemetry()

    # ------------------------------------------------------------------
    def _publish_telemetry(self):
        for dev in (1, 2, 3, 4, 5):
            self.bus.push_telemetry(
                SimCanBus.make_rpm_frame(dev, self.actual_rpm[dev]))
        self.bus.push_telemetry(SimCanBus.make_linear_pos_frame(self.odometry["left"]))
        self.bus.push_telemetry(SimCanBus.make_sabertooth_frame(self.act_adc, self.depo_cm))
        self.bus.push_telemetry(SimCanBus.make_imu_frame(math.degrees(self.yaw)))

    # ------------------------------------------------------------------
    # Perception feeds (ZMQ json like Rover/Drivers/perception.py)
    # ------------------------------------------------------------------
    def front_camera_scan(self):
        """16-band forward scan -> list of {rel_x, rel_z, type} detections
        in the rover frame (rel_z = forward, rel_x = right), matching
        what perception.py publishes as point_cloud obstacles."""
        detections = []
        obstacles = ([{**c, "type": "crater"} for c in self.arena.craters] +
                     [{**b, "type": "rock"} for b in self.arena.boulders])
        walls = [
            (0.0, 0.0, ARUCO_X, 0.0),
            (0.0, ARENA_W, ARUCO_X, ARENA_W),
            (0.0, 0.0, 0.0, ARENA_W),
            (ARUCO_X, 0.0, ARUCO_X, ARENA_W),
        ]
        half_fov = math.radians(FRONT_FOV_DEG / 2.0)
        for band in range(NUM_BANDS):
            frac = (band + 0.5) / NUM_BANDS
            angle = self.yaw + half_fov - frac * 2.0 * half_fov
            dx, dz = math.sin(angle), math.cos(angle)
            best, best_type = None, None
            for obs in obstacles:
                t = ray_circle_intersect(self.x, self.z, dx, dz,
                                         obs["x"], obs["z"], obs["r"])
                if t is not None and FRONT_MIN_DIST < t < FRONT_MAX_DIST:
                    if best is None or t < best:
                        best, best_type = t, obs["type"]
            for w in walls:
                t = ray_segment_intersect(self.x, self.z, dx, dz, *w)
                if t is not None and FRONT_MIN_DIST < t < FRONT_MAX_DIST:
                    if best is None or t < best:
                        best, best_type = t, "wall"
            if best is not None:
                hx, hz = self.x + dx * best, self.z + dz * best
                ox, oz = hx - self.x, hz - self.z
                detections.append({
                    "rel_x": ox * math.cos(self.yaw) - oz * math.sin(self.yaw),
                    "rel_z": ox * math.sin(self.yaw) + oz * math.cos(self.yaw),
                    "type": best_type,
                    "hit_x": hx, "hit_z": hz,
                })
        return detections

    def rear_aruco(self):
        """Rear ArUco camera. Returns perception.py-style payload fields:
        aruco_pos = [tvec_x, tvec_y, tvec_z, yaw] when locked."""
        rear_yaw = self.yaw + math.pi
        dx_a, dz_a = ARUCO_X - self.x, ARUCO_Z - self.z
        dist = math.hypot(dx_a, dz_a)
        diff = (math.atan2(dx_a, dz_a) - rear_yaw + math.pi) % (2 * math.pi) - math.pi
        visible = abs(diff) < math.radians(REAR_FOV_DEG / 2.0) \
            and REAR_MIN_DIST < dist < REAR_MAX_DIST
        if not visible:
            return {"localization_mode": "BLIND (ENC)", "aruco_pos": []}
        fwd_x, fwd_z = math.sin(rear_yaw), math.cos(rear_yaw)
        right_x, right_z = fwd_z, -fwd_x
        tvec_z = dx_a * fwd_x + dz_a * fwd_z
        tvec_x = dx_a * right_x + dz_a * right_z
        return {
            "localization_mode": "ARUCO_LOCKED",
            "aruco_pos": [tvec_x, 0.0, tvec_z, self.yaw],
        }

    def telemetry_payload(self):
        payload = {
            "vo_status": "OK",
            "obstacles": [{"type": "point_cloud", "rel_x": d["rel_x"],
                           "rel_z": d["rel_z"]}
                          for d in self.front_camera_scan()],
        }
        payload.update(self.rear_aruco())
        return payload

    def snapshot(self):
        return {
            "x": round(self.x, 4), "z": round(self.z, 4),
            "yaw": round(self.yaw, 4), "yaw_deg": round(math.degrees(self.yaw), 2),
            "wheel_rpm": {str(k): round(v, 2) for k, v in self.actual_rpm.items()},
            "encoder_distance_m": {k: round(v, 4) for k, v in self.odometry.items()},
            "depth_cm": round((self.act_adc - CAL_B) / CAL_M, 3),
            "depth_adc": int(self.act_adc),
            "depo_cm": round(self.depo_cm, 3),
            "payload_kg": round(self.load_kg, 3),
        }

