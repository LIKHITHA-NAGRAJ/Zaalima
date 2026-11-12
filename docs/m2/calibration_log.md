# M2 Calibration Log

Platform: Tinkercad Circuits | L293D | 9V external supply | cruising PWM = 200

## 1. Model parameters (drive the dead-reckoning and the PID plant)
| Parameter | Value | Source |
|---|---|---|
| Wheel diameter | 65 mm | Chosen wheel size for the chassis |
| Wheel base | 120 mm | Chosen chassis width |
| Motor speed at PWM 255 | 100 rpm | Typical gear-motor figure; edit if your datasheet differs |

Derived (formulas in the report):
| Quantity | Value at PWM 200 |
|---|---|
| Forward speed | 266.9 mm/s |
| Tank-turn rate (wheels counter-rotate) | 254.9 deg/s |
| Pivot rate (one wheel) | 127.4 deg/s |
| Time for 1000 mm straight | 3.75 s |
| Time for an ideal 90 deg tank turn | 0.35 s |

## 2. Drift compensation (static trim)
A drift of the right wheel by factor f gives open-loop heading error even on a "straight" command.
Set the potentiometer on A0 to the factor, run `k` (500 mm open loop), and read the pose with `q`.

| Right wheel factor f | Open-loop result after 1000 mm (y mm / heading deg) | Trim used | After trim (y mm / heading deg) |
|---|---|---|---|
| 1.00 | 0.6 / 0.07 | 1.00 | - |
| 1.10 | 381.4 / 45.7 | 0.909 | |
| 1.15 | 525.4 / 66.9 | 0.870 (1/1.15) | 0.0 / 0.00 |
| 0.85 | -583.7 / -77.6 | 1.176 (1/0.85) | |

Values above come from running this firmware logic offline. Re-run in Tinkercad and write what you see
in the blank cells. Trim = 1 / f. Set `RIGHT_TRIM` (or `LEFT_TRIM` if f < 1 is easier to correct on the left).

## 3. Final constants committed
- `LEFT_TRIM  = 1.00`
- `RIGHT_TRIM = 1.00` (set to 1/f once the team agrees a nominal drift)
- `KP = 250`, `KI = 400`, `KD = 12`
- `U_MAX = 80`, `I_MAX = 40`

## 4. Notes
- With PID on, residual drift is removed in about 1 s, so trims mainly reduce the load on the PID.
