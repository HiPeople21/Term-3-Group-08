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

- [FLOWCHARTS1.md](FLOWCHARTS1.md) — Main loop and stage transitions
- [FLOWCHARTS2.md](FLOWCHARTS2.md) — Base exit logic, line following and planting mission
- [FLOWCHARTS3.md](FLOWCHARTS3.md) — Kill switch and safety handling, WiFi/MQTT communication

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

<!-- Fill in your calibration notes below -->

### IR Array Calibration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Number of sensors | 11 | |
| Calibration sweeps | 400 | |
| Calibration method | | _e.g. manual sweep across black line on white surface_ |
| Surface type | | _e.g. white board with black electrical tape_ |
| Min/max values observed | | _e.g. min ~50, max ~2500_ |

### Encoder / Distance Calibration

| Parameter | Value | How it was measured |
|-----------|-------|---------------------|
| Counts per revolution | 1400 | |
| Wheel diameter | 38.5 mm | |
| Track distance (between tracks) | 170 mm | |
| IR-to-hole distance | 116.5 mm / 1355 ticks | |
| Turn correction factor | 4.75/4 | |
| Ticks for 60° planter rotation | 233 | |

### PID Tuning

| Controller | Kp | Ki | Kd | Notes |
|------------|----|----|-----|-------|
| Line following | 1.0 | 0.0 | 0.0 | |
| Wall following (control_test) | 2.5 | 0.0 | 2.0 | |

## Testing Evidence

<!-- Add rows for each test you've run. Link to photos/videos if available. -->

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

<!-- Fill in what didn't work or is incomplete -->

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
