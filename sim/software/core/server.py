"""Localhost TCP/JSON control server for the Flutter GUI.

Protocol: newline-delimited JSON. Every request gets one JSON response.

Requests (from Flutter):
  {"cmd": "state"}                          -> full sim state snapshot
  {"cmd": "tick", "dt": 0.016, "speed": 2, "manual_key": "w"|null}
                                            -> advance sim, returns state
  {"cmd": "toggle_edit"}                    -> {"edit_mode": true|false}
  {"cmd": "add_element", "type": "crater"|"boulder", "x": .., "z": .., "r": ..}
  {"cmd": "remove_element", "type": .., "index": ..}
  {"cmd": "move_element", "type": .., "index": .., "x": .., "z": ..}
  {"cmd": "randomize", "seed": int|null}
  {"cmd": "reset_default"}
  {"cmd": "set_speed", "speed": number}
  {"cmd": "export_test_json"}               -> runs deterministic script,
                                               writes sim_test/*.json,
                                               returns file path + summary

The engine is single-threaded behind a lock; ticks are driven by the GUI.
"""

import json
import os
import socket
import threading
import time

from .arena import Arena
from .engine import SimEngine

SIM_TEST_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "sim_test"))


class SimServer:
    def __init__(self, host="127.0.0.1", port=8745):
        self.engine = SimEngine()
        self.lock = threading.Lock()
        self.edit_mode = False
        self.speed = 1.0
        self.host, self.port = host, port
        self._srv = None
        self._thread = None
        self.running = False

    # ------------------------------------------------------------------
    def state(self):
        s = self.engine.sensors
        return {
            "ok": True,
            "sim_time": round(self.engine.sim_time, 3),
            "running_script_action": self.engine.current_action()[0]
                if self.engine.current_action() else None,
            "script_index": self.engine.script_index,
            "edit_mode": self.edit_mode,
            "speed": self.speed,
            "rover": s.snapshot(),
            "arena": self.engine.arena.to_dict(),
            "telemetry": s.telemetry_payload(),
        }

    # ------------------------------------------------------------------
    def handle(self, req):
        cmd = req.get("cmd")
        with self.lock:
            if cmd == "state":
                return self.state()
            if cmd == "tick":
                dt = float(req.get("dt", 1 / 60)) * self.speed
                key = req.get("manual_key")
                self.engine.step(dt, manual_key=key)
                return self.state()
            if cmd == "toggle_edit":
                self.edit_mode = not self.edit_mode
                return {"ok": True, "edit_mode": self.edit_mode}
            if cmd == "set_speed":
                self.speed = float(req.get("speed", 1.0))
                return {"ok": True, "speed": self.speed}
            if cmd == "add_element":
                el = {"x": float(req["x"]), "z": float(req["z"]),
                      "r": float(req.get("r", 0.2))}
                lst = self.engine.arena.craters if req["type"] == "crater" \
                    else self.engine.arena.boulders
                lst.append(el)
                return {"ok": True, "index": len(lst) - 1}
            if cmd == "remove_element":
                lst = self.engine.arena.craters if req["type"] == "crater" \
                    else self.engine.arena.boulders
                i = int(req["index"])
                if 0 <= i < len(lst):
                    lst.pop(i)
                return self.state()
            if cmd == "move_element":
                lst = self.engine.arena.craters if req["type"] == "crater" \
                    else self.engine.arena.boulders
                i = int(req["index"])
                if 0 <= i < len(lst):
                    lst[i]["x"] = float(req["x"])
                    lst[i]["z"] = float(req["z"])
                return self.state()
            if cmd == "randomize":
                seed = req.get("seed")
                self.engine.arena.randomize(seed=None if seed is None else int(seed))
                return self.state()
            if cmd == "reset_default":
                self.engine.reset()
                return self.state()
            if cmd == "export_test_json":
                return self.export_test_json()
            return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    # ------------------------------------------------------------------
    def export_test_json(self):
        """Run the hard-coded deterministic script from a fresh default
        arena and dump rover position + sensor output data to sim_test/."""
        eng = SimEngine(Arena())          # always deterministic defaults
        eng.run_script(dt=1.0 / 60.0)
        data = eng.export_json()
        os.makedirs(SIM_TEST_DIR, exist_ok=True)
        fname = time.strftime("sim_output_%Y%m%d_%H%M%S.json")
        path = os.path.join(SIM_TEST_DIR, fname)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        return {
            "ok": True,
            "path": path,
            "summary": {
                "steps_recorded": len(eng.history),
                "final_rover_state": eng.sensors.snapshot(),
                "aruco_locked": eng.sensors.rear_aruco()["localization_mode"],
            },
        }

    # ------------------------------------------------------------------
    def _client(self, conn):
        buf = b""
        conn.settimeout(30)
        try:
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        req = json.loads(line)
                        resp = self.handle(req)
                    except Exception as e:      # keep server alive
                        resp = {"ok": False, "error": repr(e)}
                    conn.sendall(json.dumps(resp).encode() + b"\n")
        finally:
            conn.close()

    def serve_forever(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(4)
        self.running = True
        print(f"[sim-core] listening on {self.host}:{self.port}")
        while self.running:
            try:
                conn, _ = self._srv.accept()
            except OSError:
                break
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def start_background(self):
        self._thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._srv:
            self._srv.close()
