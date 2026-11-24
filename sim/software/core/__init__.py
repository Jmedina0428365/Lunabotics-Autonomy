"""Core simulation package for the Lunabotics rover sim.

Modules:
  arena.py   - guidebook arena geometry (8.10 x 4.57 m, berm, ArUco)
  can_sim.py - simulated CAN bus / frame codecs matching UMRS main.rs
  sensors.py - synthetic wheel encoders, IMU, cameras, linear actuators
  engine.py  - deterministic hard-coded-script simulation engine
  server.py  - localhost TCP/JSON API consumed by the Flutter GUI
"""
