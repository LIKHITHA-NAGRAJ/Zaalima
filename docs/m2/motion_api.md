# Motion Module API (M2)

Branch: `feature/m2-motion` | File: `firmware/motion/motion.ino`

Tinkercad allows one sketch per Arduino, so M1 pastes this module into the integrated sketch.
Everything below is what the other modules need to know to avoid pin, name and timing clashes.

## Pins used by M2
| Function | Arduino pin |
|---|---|
| Left motor enable (PWM) / IN1 / IN2 | 5 / 4 / 7 |
| Right motor enable (PWM) / IN3 / IN4 | 6 / 8 / 12 |
| Disturbance potentiometer (optional, simulation only) | A0 |

Pins **2, 3, 9, 10, 11** are left free for M3 (HC-SR04 echo interrupt, trigger, servo). Pin 11 is also
free. Note that the Servo library disables PWM on pins 9 and 10, which M2 does not use.

## Functions other modules may call
| Function | Purpose |
|---|---|
| `moveMM(float mm)` | Non-blocking straight move (negative = reverse). Uses heading-hold PID when `pidEnabled` is true. |
| `turnByDeg(float deg)` | Non-blocking in-place tank turn. Positive = left (counter-clockwise), negative = right. Stops within 1.5 degrees. |
| `bool motionBusy()` | True while any move is running. Poll it from the state machine; never wait with `delay()`. |
| `abortMotion()` | Immediate stop. Use it for EMERGENCY_STOP. |
| `stopRover()` | Set both motors to zero. |
| `forward(s) reverse(s) pivotLeft(s) pivotRight(s) tankLeft(s) tankRight(s)` | Raw open-loop drive at PWM `s` (0-255). |
| `startMotion(m, speed, ms)` | Timed open-loop motion. `m` is one of `M_FWD M_REV M_PIV_L M_PIV_R M_TANK_L M_TANK_R M_STOP`. |
| `getPose(x, y, th)` | Dead-reckoned position in mm and heading in radians (for M4's occupancy map). |
| `resetPose()` | Zero the pose (start of a mapping run). |

## Pose convention (agree with M4)
- Origin where the rover started, **+x forward**, **+y to the left**.
- Heading `th` in radians, 0 along +x, positive counter-clockwise (left turn).
- To place a scan point at servo angle `a` (relative to rover front, left positive) and distance `r` in mm:
  `px = x + r*cos(th + a)`, `py = y + r*sin(th + a)`.
- Update the pose every 20 ms (`CONTROL_MS`) from inside `loop()`. Do not call `getPose()` from an interrupt.

## Global names M1 should keep unique when merging
`ENA ENB IN1 IN2 IN3 IN4 POT_PIN mode currentMotion cmdL cmdR rightFactor poseX poseY poseTh travelled targetDist targetTh straightDir pidInt lastTh lastU KP KI KD pidEnabled plotMode driveSpeed LEFT_TRIM RIGHT_TRIM`
and the functions listed above, plus `setMotor drive wrapPi pwmToSpeed readDisturbance updatePose controlStep handleSerial runDemo printPose`.

If M3's sketch also reads Serial characters, give each module its own command letters (M2 uses `f b l r a d s h k t u g q z p + -`).

## Timing rules
- No `delay()` anywhere. `loop()` must run freely so `controlStep()` fires every 20 ms.
- M3's ultrasonic read should be non-blocking or the pose integration will lag during scans. If a scan must
  stop the rover, call `abortMotion()` first (the pose estimate stays correct because no motion is commanded).

## Typical use from M3's state machine
```cpp
case SCANNING:   /* M3 scan code */                      break;
case MOVING:     if (!motionBusy()) moveMM(200);         break;   // 200 mm step
case EVADING:    if (!motionBusy()) turnByDeg(90);       break;   // evasive pivot
case EMERGENCY_STOP: abortMotion();                      break;
```

## Simulation note
Tinkercad has no physics. The module integrates the commanded wheel speeds (a software plant model),
and the A0 potentiometer scales the right wheel speed from 0.85x to 1.15x to simulate wheel slip. On a
real robot, replace `updatePose()` with gyro/encoder feedback and set `USE_DISTURBANCE_POT` to 0.
