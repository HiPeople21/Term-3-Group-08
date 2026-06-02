# MEng Robotics and Artificial Intelligence Robotics Challenge 2026, Group 8

## Setup

### Installations

In the Arduino IDE, go to Boards Manager, and install `Arduino Mbed OS Giga Boards` by Arduino. 

Then, go to Library Manager and install the following:
- `MFRC522_I2C` by kkloesener
- `MiniMessenger` by Ziwen Lu, and Alex Charitonidis
- `Motoron` by Pololu
- `QTRSensors` by Pololu

Note that the `QTRSensors` library must be modified before running the code. In `Arduino/libraries/QTRSensors/QTRSensors.cpp`, all the `interrupts()` and `noInterrupts()` function calls must be removed or commented out. This is because these functions cause the Arduino Giga R1 to crash when trying to start up due to its dual core nature.

### Uploading and Running

Plug the Arduino into your computer using a data transmitting USB cable. There are two ways to select the Arduino:
- Click on the dropdown in the top left of the Arduino IDE, to the right of the debugging button, and select the Arduino. It should say something like `Arduino Giga R1` with `COMX` under it, where X is an integer.
- Click on `Tools` in the menubar. Select `Arduino Giga R1` in Board, and `COMX` in Port.

After this, you can click `Verify` to ensure the code compiles correctly. Click `Upload` to begin the uploading process. Once the sketch has been uploaded, you can unplug the Arduino. Then the Arduino begins its calibration process for the IR array.

## Repository Structure

In this repository, there are many folders suffixed with `_test`. These contain sketches that either test individual components, or were set up in advance to a trial run, such as the mechanical design trial.

Additionally, the `checklists/` directory contain the programming and control checklists for easy access.

The `main/` directory is where the code used in the finals will live. In this directory there are many files:
- `main.ino`: the main sketch which runs the robot.
- `secrets.h`: contains sensitive and configuration data such as the WiFi SSID and password, essentially acting as a `.env` file.
- `motors.cpp`: contains functions for motor usage and control.
- `motors.h`: header file for `motors.cpp`.
- `parser.cpp`: file containing a utility function which parses the server response string into a hashmap mapping a string to a string, where "a=1 b=2 c=3" would become `{{"a", "1"}, {"b", "2"}, {"c", "3"}}`.
- `parser.h`: header file for `parser.cpp`.
- `sensors.cpp`: contains functions for obtaining sensor readings.
- `sensors.h`: header file for `sensors.cpp`.
- `wifi_utils.cpp`: handles WiFi communication with the server.
- `wifi_utils.h`: header file for `wifi_utils.cpp`.

## Software Overview

See the flowchart documents for detailed diagrams of each behaviour:

- [FLOWCHARTS1.md](FLOWCHARTS1.md) - Main loop and stage transitions
- [FLOWCHARTS2.md](FLOWCHARTS2.md) - Base exit logic, line following and planting mission
- [FLOWCHARTS3.md](FLOWCHARTS3.md) - Kill switch and safety handling, WiFi/MQTT communication

### Component Interaction

| Component | Communicates With | Purpose |
|-----------|------------------|---------|
| `main.ino` | All modules | State machine, loop orchestration |
| `motors.cpp` | Motoron (I2C, 0x12) | Track motors, planter motor, encoder reading, point turns |
| `sensors.cpp` | IR array (GPIO), TOF (Serial1/4), Ultrasonic (GPIO) | Line position, distance sensing |
| `wifi_utils.cpp` | MQTT server via MiniMessenger | Kill switch, fertility checks, airlock requests, heartbeat |
| `parser.cpp` | `wifi_utils.cpp` | Parses `key=value` server messages into a map |
| RFID (MFRC522) | I2C (Wire1, 0x28) | Tag identification at grid nodes and base exit |

### Key Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `baseSpeed` | `500 * 6/7.2` | Nominal PWM scaled for 7.2V battery (6V motor rating) |
| `maxSpeed` | `800 * 6/7.2` | Maximum PWM, voltage-compensated |
| `setpoint` | 5500 | PID target (centre of 11-sensor IR array, range 0–11000) |
| `ticksToHole` | 1355 | Encoder ticks from IR detection to planting hole |
| `ticksToPlant` | 233 | Encoder ticks for one 60° planter rotation |
| `debounceDelay` | 50 ms | Kill switch button debounce threshold |
| `HEARTBEAT_TIMEOUT_MS` | 1000 ms | Server heartbeat timeout before auto-kill |
| `IR_WINDOW_MS` | 1000 ms | Time window to find RFID after IR hole detection |
| `FERTILITY_TIMEOUT_MS` | 5000 ms | Timeout waiting for server fertility response |

## Calibration


### IR Array Calibration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Number of sensors | 11 | We originally wanted to use the given 9 channel IR array with two additional 2 channel IR array at a wider distance to better handle PID control, since this would mean the robot would oscillate less to try find the line. However, one of the edge sensors on the 9 channel IR array wasn't working, so we removed it from the code. We also decided to remove the sensor from the opposite side to balance the IR sensors on each side. |
| Calibration sweeps | 400 | This was the value used in the example in the official `QTRSensors` repository. The file can be found [here](https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRRCExample/QTRRCExample.ino) |
| Calibration method | | Manual sweep across black line on the arena surface. |
| Min/max values observed | | _e.g. min ~50, max ~2500_ |

### Encoder / Distance Calibration

| Parameter | Value | How it was measured |
|-----------|-------|---------------------|
| Counts per revolution | 1400 | When we were testing the planter mechanism, we had try many values for the encoder ticks for 360°. 1200 was too little, but 1600 was too much. Eventually, we narrowed it down to 1400, which worked very well. A week or two later, we learnt that the motor was made by DFRobot, and when we checked their website it said "average output number of pulses can reach up to 7*2*100 pulses per revolution". |
| Wheel diameter | 38.5 mm | Vernier caliper |
| Track distance (between tracks) | 170 mm | Vernier caliper |
| IR-to-hole distance | 116.5 mm / 1355 ticks | Vernier caliper |
| Turn correction factor | 4.75/4 | When testing discrete 90° turning for the robot, we found that running it 5 times led to an approximate 360° turn. Hence, we added the scale factor. 5/4 was too high, 4.8/4 was still slightly too high, and 4.75/4 landed very close to 90°. |
| Ticks for 60° planter rotation | 233 | 1400 / 6 = 233.3333... ≈ 233 |

### PID Tuning

| Controller | Kp | Ki | Kd | Notes |
|------------|----|----|-----|-------|
| Line following | 1.0 | 0.0 | 0.0 | |
| Wall following (control_test) | 2.5 | 0.0 | 2.0 | |

## Testing Evidence

| Date | Test | Result | Notes |
|------|------|--------|-------|
| | IR calibration | | |
| | Line following (straight) | | |
| | Line following (curves) | | |
| | Junction detection | | |
| | RFID reading | | |
| | Airlock open request | | |
| | Planter rotation | | |
| | Kill switch (hardware) | | |
| | Kill switch (WiFi) | | |
| | Heartbeat timeout | | |
| | Encoder-based driving | | |
| | Point turn accuracy | | |

## Known Limitations


| Feature | Status | Notes |
|---------|--------|-------|
| Line following | | |
| RFID + planting | | |
| Base exit (airlock) | | |
| Kill switch (local + WiFi) | | |
| TOF distance sensors | | |
| Wall following | | |
| Obstacle avoidance | | |
| Return to base | | |
| Dead reckoning | | |
