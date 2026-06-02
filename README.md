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

## Calibration
