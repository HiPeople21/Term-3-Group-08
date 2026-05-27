# IR Array Integration Design

**Date:** 2026-05-21  
**Scope:** Integrate 12-sensor QTR IR array from `ir_array_test/` into `main/`

---

## Goal

Add IR array sensor reading and calibration to `main/` so the robot can print line position and error values during operation. No autonomous motor control — read + print only.

---

## Architecture

The IR array logic is added to the existing `sensors.h` / `sensors.cpp` module, consistent with how TOF and ultrasonic sensors are already handled. Two new functions are introduced:

- `initIRArray()` — configures QTR sensor pins and runs blocking calibration (~10s) during startup
- `readIRArray()` — non-blocking read called every 100ms in the main loop, prints sensor values, position, and error

`main.ino` gains two call sites: one in `setup()` and one in `loop()`.

---

## Sensor Configuration

- **Library:** QTRSensors (RC mode)
- **Sensor count:** 12
- **Pins:** `{31, 30, 27, 36, 23, 28, 29, 24, 37, 22, 33, 32}` (defined globally to avoid GIGA memory crash)
- **Center position:** 6000 (matches `ir_array_test` reference value)
- **Error:** `|6000 - position|`

---

## Changes

### `sensors.h`
Add two declarations:
```cpp
void initIRArray();
void readIRArray();
```

### `sensors.cpp`
At the top:
```cpp
#include <QTRSensors.h>
```

New globals (file-scope):
```cpp
static QTRSensors qtr;
static const uint8_t kSensorCount = 12;
static const uint8_t kSensorPins[12] = {31, 30, 27, 36, 23, 28, 29, 24, 37, 22, 33, 32};
static uint16_t sensorValues[12];
```

`initIRArray()`:
- Calls `qtr.setTypeRC()` and `qtr.setSensorPins(kSensorPins, kSensorCount)`
- Turns `LED_BUILTIN` on, runs 400 calibration iterations, turns it off
- Prints MIN and MAX calibration values to Serial

`readIRArray()`:
- Non-blocking: uses `static unsigned long lastIRRead` + 100ms guard (same pattern as `readUltrasonic()`)
- Calls `qtr.readLineBlack(sensorValues)`
- Prints tab-separated sensor values, position, and `|6000 - position|` error

### `main.ino`
- Add `initIRArray()` call in `setup()` after `initSensors()`
- Uncomment / add `readIRArray()` call in `loop()`

---

## Calibration Behaviour

Calibration blocks `setup()` for ~10 seconds. During this window the user sweeps all 12 sensors across the line. `LED_BUILTIN` is HIGH during calibration and LOW when done. Serial prints MIN/MAX tables on completion. This matches the `ir_array_test` behaviour exactly.

---

## What Is Not Changing

- No PID or proportional motor control
- No new files — IR code lives in `sensors.cpp` / `sensors.h`
- No changes to motor, RFID, WiFi, or kill-switch logic
