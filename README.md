# Zaalima - Project 1: Autonomous SLAM-Inspired Occupancy Mapping Rover

Group 11 | Zaalima Development IoT & Robotics Internship | Simulated in Autodesk Tinkercad Circuits

## Overview

Real LiDAR modules are hard to model in a browser simulator, so this rover imitates one: an HC-SR04
ultrasonic sensor sits on a servo that pans through 180 degrees. Each distance reading (distance `r`,
servo angle `theta`) is converted to 2D coordinates and plotted as an obstacle map. A differential-drive
rover (Arduino Uno, L293D, two DC gear motors) moves, detects obstacles and turns toward the clearer side.

**Working loop**

1. **Scan:** the servo points the sensor ahead (90 degrees) and measures the distance.
2. **Decide:** if the distance is more than 25 cm the rover drives forward. Otherwise it stops, scans left
   (150 degrees) and right (30 and 15 degrees) and turns toward the clearer side.
3. **Move:** the L293D drives the motors (forward, turn left, turn right, stop).
4. **Map:** every reading is printed over serial as `Angle: <deg> degrees | Distance: <cm> cm` and converted
   with `x = r*cos(theta)`, `y = r*sin(theta)` by a Python script that draws the 2D map.

## Team

Likhitha N, Vasudha Vairagade, Rochana Udagatti, Saleha Shaikh

| Member | Role | Responsibility |
|---|---|---|
| M1 | Team Lead, Hardware Integration | Circuit, wiring schedule, GitHub, integration |
| M2 | Motion Control Engineer | Motor control, dead reckoning, PID heading control |
| M3 | Sensing and Navigation Engineer | Servo scanning, HC-SR04, obstacle avoidance, maze testing |
| M4 | Mapping and Data Engineer | Coordinate conversion, serial data, Python map |

## Repository structure

```
Zaalima/
|-- README.md
|-- .gitignore
|-- firmware/
|   `-- motion/
|       `-- motion.ino                  M2: motion control, pose estimate, PID heading hold
|-- mapping/                            M4: mapping and data
|   |-- README.md, report.md
|   |-- plot_points.py                  reads serial_capture.txt and plots the 2D map
|   |-- serial_capture.txt              sample output from the Tinkercad Serial Monitor
|   |-- polar_to_xy_test.ino.txt        on-board polar-to-Cartesian test (13 angles, 0-180)
|   `-- points.csv                      older sample file (empty, not used)
`-- docs/
    |-- m2/                             M2 documentation
    |   |-- motion_api.md               pins, functions and pose convention for other modules
    |   |-- calibration_log.md          model parameters, derived speeds, drift and trim table
    |   |-- kinematics_report.pdf/.md   kinematics, dead reckoning and PID report
    |   `-- screenshots/                circuit, schematic, serial monitor, output, PID graph
    `-- screenshots/
        |-- code.txt                    main rover sketch: scan, decide, drive
        |-- Wiring_tables_of_components.docx   wiring connection schedule
        `-- full_circuit.png.png        full Tinkercad circuit
```

## Hardware and pin map

| Part | Connection |
|---|---|
| L293D ENA (pin 1) / IN1 (pin 2) / IN2 (pin 7) | Arduino D5 / D4 / D7 (left motor) |
| L293D ENB (pin 9) / IN3 (pin 10) / IN4 (pin 15) | Arduino D6 / D8 / D12 (right motor) |
| L293D OUT1, OUT2 / OUT3, OUT4 | Left motor / right motor |
| L293D VCC1 (pin 16) | Arduino 5V |
| L293D VCC2 (pin 8) | External DC motor supply (+), not the Arduino 5V pin |
| HC-SR04 TRIG / ECHO | Arduino D2 / D13 |
| Servo signal | Arduino D3 |
| Potentiometer (optional, motion.ino only) | A0 (simulated right-wheel disturbance) |
| Ground | Arduino GND, L293D grounds, sensor, servo and supply negative on one common ground |

Full schedule: `docs/screenshots/Wiring_tables_of_components.docx`.

## How to run

**Rover (scan and obstacle avoidance).** Build the circuit in Tinkercad, paste `docs/screenshots/code.txt`
into the code editor, start the simulation and open the Serial Monitor at 9600 baud. Drag the HC-SR04
obstacle dot to trigger avoidance.

**Motion module (M2).** Paste `firmware/motion/motion.ino`, start the simulation and send one character
in the Serial Monitor:

| Command | Action |
|---|---|
| `f` `b` `l` `r` `a` `d` | Timed forward, reverse, pivot left/right, tank left/right |
| `h` / `k` | Straight 500 mm with / without PID heading hold |
| `t` / `u` | Turn +90 / -90 degrees |
| `g` `s` | Demo sequence / stop |
| `q` `z` `p` | Print pose / zero pose / toggle Serial Plotter mode |
| `+` `-` | Change PWM |

**Map.** Copy the Serial Monitor output into `mapping/serial_capture.txt`, then run:

```
pip install matplotlib
python mapping/plot_points.py
```

Readings above 200 cm mean nothing was detected and are ignored.

## Subsystems

- **Motion (M2):** differential-drive kinematics `v = (vR + vL)/2`, `omega = (vR - vL)/L`; non-blocking moves
  (`moveMM`, `turnByDeg`, `motionBusy`, `abortMotion`); dead-reckoned pose through `getPose()`; PID heading
  control (KP 250, KI 400, KD 12). In the control-logic model, a right wheel running 15% fast drifts about
  525 mm over 1 m without PID and about 10 mm with PID. See `docs/m2/kinematics_report.pdf`.
- **Sensing and navigation (M3):** servo scan at 90, 150, 30 and 15 degrees and a pick-the-clearer-side
  decision, as in `docs/screenshots/code.txt`.
- **Mapping (M4):** polar-to-Cartesian conversion, range filtering and a 2D scatter map. See `mapping/README.md`.

## Current status and known issues

| Item | Status |
|---|---|
| Motion module and documentation | Done, with Tinkercad screenshots in `docs/m2/screenshots/` |
| Scan and obstacle-avoidance sketch | Working in Tinkercad (logged LEFT/RIGHT decisions) |
| Map from serial data | Obstacle points plotted; free/unknown grid cells not yet drawn |
| Full 15 degree sweep log | Pending |
| Integration | The main sketch drives the motors directly and uses blocking `delay()`; the motion module is not merged into it yet |
| Maze and stress-test results | Pending |

Notes for integration: M2 uses x forward, y left, heading counter-clockwise; the mapping script uses
servo angle with 90 degrees ahead, x sideways and y forward. The conversion is in `docs/m2/motion_api.md`.

## Workflow (Zaalima SOP v1.1)

Each member commits to their own branch (`feature/m1-integration`, `feature/m2-motion`,
`feature/m3-navigation`, `feature/m4-mapping`); the team lead merges to `main`. The Mid Review needs commits
on at least 10 different days in the 14 days before it, and the Final Review needs a commit on every day of
the 20 days before it.

## Deliverables

Autonomous SLAM Rover firmware and kinematics report, 2D occupancy-grid telemetry from the Tinkercad serial
buffer, and the firmware and mapping documentation. The combined project report goes in `report/`.
