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
   (150 degrees) and right (30 and 15 degrees) and turns toward the clearer side for 0.6 s. If neither side
   is clear it stays stopped.
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
|-- M3_navigation.ino                   M3: interrupt-driven 15 degree sweep, FSM, SIM mode (own pin map, see Known issues)
|-- firmware/
|   `-- motion/
|       `-- motion.ino                  M2: motion control, pose estimate, PID heading hold
|-- mapping/                            M4: mapping and data
|   |-- README.md, report.md
|   |-- plot_points.py                  reads serial_capture.txt and plots the 2D map
|   |-- serial_capture.txt              output from the Tinkercad Serial Monitor (35 readings)
|   |-- polar_to_xy_test.ino.txt        on-board polar-to-Cartesian test (13 angles, 0-180)
|   `-- points.csv                      older sample file (empty, not used)
|-- docs/
|   |-- m2/                             M2 documentation
|   |   |-- motion_api.md               pins, functions and pose convention for other modules
|   |   |-- calibration_log.md          model parameters, derived speeds, drift and trim table
|   |   |-- kinematics_report.pdf/.md   kinematics, dead reckoning and PID report
|   |   `-- screenshots/                circuit, schematic, serial monitor, output, PID graph
|   `-- screenshots/
|       |-- code.txt                    main rover sketch: scan, decide, drive
|       |-- Wiring_tables_of_components.docx   wiring connection schedule
|       `-- full_circuit.png.png        full Tinkercad circuit
`-- report/                             final project report (PDF)
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

**Navigation FSM (M3).** Paste `M3_navigation.ino` into a circuit wired as in its header comment
(servo D9, TRIG D7, ECHO D2). Send `x` in the Serial Monitor to toggle SIM mode and `0` to `4` to pick a
virtual scan scenario, because Tinkercad's HC-SR04 returns one distance whatever the servo angle.

**Map.** Copy the Serial Monitor output into `mapping/serial_capture.txt`, then run:

```
pip install matplotlib
python mapping/plot_points.py
```

Readings above 200 cm mean nothing was detected and are ignored.

**On-board conversion test.** Paste `mapping/polar_to_xy_test.ino.txt` into Tinkercad and open the Serial
Monitor. It prints `x,y` for 13 placeholder distances at 0, 15, ... 180 degrees. The 333 cm value at 165
degrees is above 200 cm and is skipped, so 12 lines are printed.

## Subsystems

- **Motion (M2):** differential-drive kinematics `v = (vR + vL)/2`, `omega = (vR - vL)/L`; non-blocking moves
  (`moveMM`, `turnByDeg`, `motionBusy`, `abortMotion`); dead-reckoned pose through `getPose()`; PID heading
  control (KP 250, KI 400, KD 12). In the control-logic model, a right wheel running 15% fast drifts about
  525 mm over 1 m without PID and about 10 mm with PID. See `docs/m2/kinematics_report.pdf`.
- **Sensing and navigation (M3):** servo scan at 90, 150, 30 and 15 degrees and a pick-the-clearer-side
  decision, as in `docs/screenshots/code.txt`. `M3_navigation.ino` extends this to a 13-position, 15 degree
  sweep with an interrupt-driven ping, edge detection and a state machine (IDLE, SCANNING, CALCULATING,
  MOVING, EMERGENCY_STOP).
- **Mapping (M4):** polar-to-Cartesian conversion, range filtering and a 2D scatter map. See `mapping/README.md`.

## Results from the captured serial run

| Item | Value |
|---|---|
| Readings in `serial_capture.txt` | 35 (15 deg: 1, 30 deg: 4, 90 deg: 26, 150 deg: 4) |
| Valid readings plotted (<= 200 cm) | 18 (30 deg: 2, 90 deg: 14, 150 deg: 2) |
| No-echo readings (about 333 cm, ignored) | 17 |
| Nearest points | 8.03 cm at 90 deg, 9.52 cm at 150 deg, 10.24 cm at 30 deg |
| Avoidance decisions logged | 4 (LEFT, LEFT, RIGHT, LEFT), each toward the side with the larger reading |

The 15 degree reading returned no echo, so it is drawn as a clear direction on the map.

## Current status and known issues

| Item | Status |
|---|---|
| Motion module and documentation | Done, with Tinkercad screenshots in `docs/m2/screenshots/` |
| Scan and obstacle-avoidance sketch | Working in Tinkercad (four logged LEFT/RIGHT decisions) |
| Map from serial data | 18 obstacle points plotted from real serial data; free/unknown grid cells not yet drawn |
| On-board polar-to-XY test | Written, 13 angles; output matches the Python conversion |
| Full 15 degree sweep log | Sketch written (`M3_navigation.ino`, SIM mode); a log from the main circuit is pending |
| Integration | The main sketch drives the motors directly and uses blocking `delay()`; the motion module is not merged into it yet |
| Pin map mismatch | `M3_navigation.ino` uses servo D9, TRIG D7, ECHO D2, while the main circuit uses servo D3, TRIG D2, ECHO D13. Align them before merging |
| Maze and stress-test results | Pending |

Notes for integration: M2 uses x forward, y left, heading counter-clockwise; the mapping script uses
servo angle with 90 degrees ahead, x sideways and y forward. The conversion is in `docs/m2/motion_api.md`.
The fourth logged decision starts at the `OBSTACLE DETECTED` line, so its front reading is not in the capture.

## Workflow (Zaalima SOP v1.1)

Each member commits to their own branch (`feature/m1-integration`, `feature/m2-motion`,
`feature/m3-navigation`, `feature/m4-mapping`); the team lead merges to `main`. The Mid Review needs commits
on at least 10 different days in the 14 days before it, and the Final Review needs a commit on every day of
the 20 days before it.

## Deliverables

Autonomous SLAM Rover firmware and kinematics report, 2D occupancy-grid telemetry from the Tinkercad serial
buffer, and the firmware and mapping documentation. The combined project report goes in `report/`.
