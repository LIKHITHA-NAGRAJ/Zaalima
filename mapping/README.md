# M4 - Mapping and Data

Part of Project 1: Autonomous SLAM-Inspired Occupancy Mapping Rover
(Zaalima IoT and Robotics internship, Tinkercad).

## What this does
Takes the rover's sensor readings (servo angle + distance), converts them
to x,y points, and draws a 2D map of the obstacles.

## Files
- plot_points.py: reads serial_capture.txt and plots the map
- serial_capture.txt: sample output copied from the Tinkercad Serial Monitor
- points.csv: older sample x,y points (not used by the current script)

## How to run
1. Install Python 3 and Matplotlib (pip install matplotlib)
2. Run: python plot_points.py

## Serial format
Current output of the circuit: Angle: 90 degrees | Distance: 17.48 cm
Proposed format: angle,distance (example: 45,87.3)
Readings above 200 cm mean "nothing detected" and are ignored.

## Maths
x = r * cos(theta), y = r * sin(theta), with theta in radians.
