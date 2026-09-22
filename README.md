# Pico Balance Bot
Hello this is my self balancing bot, which is a a two-wheeled self-balancing robot driven by a Raspberry Pi Pico 2, using a PID controller on the tilt angle from an MPU-92/65 IMU.

Hardware:

- Raspberry Pi Pico 2
- MPU-9265 IMU breakout
- L298N dual H-bridge motor driver
- 2 × TT gear motors with wheels
- Battery pack for the motors + a switch

Wiring:
From the IMU sensor to Pico 2: 
VCC to 3V3(OUT), pin 36 (3.3V) 
MPU GND to GND 
MPU SDA to GP4 (pin 6) 
MPU SCL to GP5 (pin 7) 
L298N ENA to GP21 
L298N IN1 to GP6 
L298N IN2 to GP7 
L298N IN3 to GP8 
L298N IN4 to GP9 
L298N ENB to GP20 
L298N GND to Pico GND (common ground is required) 

Removed the ENA/ENB jumper caps on the L298N so the PWM pins control speed. Kept the 5V-EN jumper on.

Software:

- Arduino IDE with the [arduino-pico](https://github.com/earlephilhower/arduino-pico) core, board set to "Raspberry Pi Pico 2"
- 'PID' library by Brett Beauregard (Library Manager)

How it works:

1. Raw accelerometer and gyro data are read directly over I2C (no DMP).
2. A complementary filter fuses them into a pitch angle: 98% integrated gyro, 2% accelerometer.
3. A PID controller drives the pitch toward the balance setpoint.
4. The output sets motor direction and PWM duty via the L298N, with a coast zone for tiny outputs and a minimum PWM to overcome motor stiction.
5. If the tilt exceeds 45°, the motors stop. If the IMU stops responding, the motors stop and the sensor is re-initialized automatically.

Calibration:

1. Hold the bot still for ~2 seconds after power-on while the gyro bias is measured.
2. Hold the bot at its true balance point and read the pitch from the Serial Monitor (115200 baud). Put that value in 'setpoint'.
3. If the wheels drive the wrong way in both fall directions, switch `REVERSE` to `DIRECT` in the PID constructor.
4. If your IMU is mounted differently, check which accelerometer axis reads ~±16384 when upright and adjust the `atan2f` axes and gyro axis to match.

 Tuning:

Start with Ki = 0 and Kd = 0. Raise Kp until the bot oscillates around upright, add Kd to damp the oscillation, then add a small Ki only if it stands but slowly drifts.

Lessons learned:
1. Clone IMUs and repeated starts: this board returned 0xFF for every register read when using 'Wire.endTransmission(false)'. Switching to a normal STOP fixed it.
2. I2C speed: 100 kHz is more reliable than 400 kHz on clone boards with long jumper wires.
3. PWM whine: the Pico's default 1 kHz PWM makes the motors audibly beep. 20 kHz removes it.
4. Center of mass matters most: with the heavy parts at axle height, the bot falls faster than the motors can catch it, and no PID gains fix that. Mass should sit high above the axle.
5. Being more patient with the wiring!!!

Current Status: 
The control chain works end to end: sensing, filtering, direction and motor response are all verified. The current chassis has a low center of mass, so the next step is raising it before final tuning.

Potential Improvements:

Mechanical:
1. Raise the center of mass. Move the battery or add weight high above the axle to slow the fall and make the bot much easier to balance.
2. Reduce frame overhang so the chassis can tilt further before touching the ground, giving the controller more room to recover.
3. Larger wheels for more ground speed per motor revolution, so the base can get back under the center of mass faster.
4. Stiffer chassis (acrylic, plywood, or 3D-printed) instead of styrofoam, to reduce flex and vibration reaching the IMU.

Electronics:
1. Replace the L298N with a MOSFET driver such as the TB6612FNG. The L298N drops 2–3 V internally, which wastes motor power and weakens corrections.
2. Soldered connections on perfboard instead of jumper wires, to eliminate the intermittent I2C dropouts caused by loose wiring.
3. Wheel encoders to measure actual wheel speed and position.
4. Battery voltage monitoring, so the controller can compensate as the battery drains and motor response weakens.

Control and Software:
1. Cascaded control loop. Add an outer loop that uses encoder data to keep the bot from drifting across the floor, feeding a small setpoint adjustment into the inner balance loop.
2. Kalman filter in place of the complementary filter for better noise handling during fast motion.
3. Integral reset when fallen. Clear the PID's accumulated integral when the tilt cutoff triggers, so a fall doesn't leave stale windup for the next attempt.
4. Live tuning over serial or Bluetooth. Adjust Kp, Ki, Kd, and setpoint without re-uploading (a Pico 2 W would allow wireless tuning).
5. Save calibration to flash so the gyro offset and setpoint persist between power cycles.

Features:
1. Obstacle avoidance using the ultrasonic sensors already mounted on the chassis.
2. Remote control for driving forward, backward, and turning while balancing.
