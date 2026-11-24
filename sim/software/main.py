#!/usr/bin/env python3
"""Rover Sim — Python simulation core entry point.

Starts the localhost control server that the Flutter GUI connects to.

Usage:
  python3 main.py                 # start server on 127.0.0.1:8745
  python3 main.py --export-test   # run deterministic script once and
                                  # write the result JSON into sim_test/
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.server import SimServer  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Lunabotics rover simulator core")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8745)
    ap.add_argument("--export-test", action="store_true",
                    help="run the hard-coded script and dump JSON to sim_test/")
    args = ap.parse_args()

    srv = SimServer(args.host, args.port)

    if args.export_test:
        result = srv.export_test_json()
        print(json.dumps(result, indent=2))
        return

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
