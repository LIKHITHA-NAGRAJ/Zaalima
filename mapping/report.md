# Firmware and Mapping Report (M4 draft)

## 1. Architecture
Arduino Uno reads the HC-SR04 sensor on a servo, prints angle and distance
over Serial, and a Python script turns those readings into a 2D map.
To add: block diagram and data flow.

## 2. Wiring
From M1's sketch: L293D ENA=5, IN1=4, IN2=7, ENB=6, IN3=8, IN4=12.
HC-SR04 TRIG=2, ECHO=13. Servo signal on pin 3.
To add: power rails and capacitor details after M1 confirms.

## 3. Kinematics and mapping maths
x = r * cos(theta), y = r * sin(theta), theta in radians.
Readings above 200 cm mean "nothing detected" and are filtered out.
To add: worked example with real readings.

## 4. Serial format and results
Current output and the proposed angle,distance format.
To add: final occupancy map plots after M3's 15-degree sweep.
