# RTOS Sensor-to-CAN Actuator Bridge
**Cairo University Eco-Racing Team (CUERT) — Embedded & Control Pre-Interview Task**

## System Architecture & Overview
This firmware implements a deterministic, safety-critical command receiver and actuator driver using FreeRTOS on an ESP32 DevKit. It models real vehicle CAN actuation nodes that receive control targets (throttle, steer, brake, ping) from a main controller over serial/CAN, process commands via an RTOS queue pipeline, drive power stages (onboard LED via PWM), and autonomously fail safe if the command link drops.
+-----------------------+
              | Serial / Terminal     |
              +-----------+-----------+
                          |
                          v
+---------------------------------------------------+
| Task 1: COMMAND_RX (Priority 4 - Highest)         |
| - Parses: THROTTLE, STEER, BRAKE, PING            |
| - Zero dynamic memory string processing           |
+-------------------------+-------------------------+
                          |
                  [ xCommandQueue ]
                          |
                          v
+---------------------------------------------------+
| Task 2: ACTUATE (Priority 3 - Medium High)        |
| - Maps THROTTLE to PWM output (GPIO 2)            |
| - STRICT BRAKE LATCH: BRAKE > 0 locks PWM to 0%   |
+---------------------------------------------------+

+---------------------------------------------------+
| Task 3: WATCHDOG (Priority 2 - Low Priority)      |
| - Checks command link freshness every 100ms       |
| - Timeout > 500ms -> Triggers 5Hz Safe-Idle Blink |
+---------------------------------------------------+

+---------------------------------------------------+
| Task 4: STATUS (Priority 1 - Lowest Priority)     |
| - Telemetry dashboard output every 1000ms         |
+---------------------------------------------------+

---

## Hardware & Toolchain Specifications
* **Target Microcontroller:** ESP32 DevKit v1
* **Framework:** Arduino / FreeRTOS (ESP32 Arduino Core v3.x API)
* **Actuator Output:** Onboard LED on Pin 2 driven via 5 kHz PWM (`ledcAttach`)
* **Communication Interface:** USB-Serial / UART @ 115200 baud (Line Ending: `Newline`)

---

## Required Interview Answers

### 1. Why did you assign the task priorities the way you did?
* **`COMMAND_RX` (Priority 4 - Highest):** Hardware serial inputs must be read immediately to prevent buffer overflows, dropping bytes, or delaying critical safety inputs such as `BRAKE`.
* **`ACTUATE` (Priority 3 - Medium High):** Motor and actuator PWM updates must execute immediately once a command is dequeued from the pipeline.
* **`WATCHDOG / FAIL-SAFE` (Priority 2 - Low):** Periodically verifies link timing without preempting active command reception or motor execution.
* **`STATUS` (Priority 1 - Lowest):** Diagnostic telemetry logging should only consume spare background CPU cycles and must never delay safety-critical execution.

### 2. Why does a stale BRAKE matter more than a stale STEER? How does the Watchdog guarantee fail-safe operation?
* **Safety Context:** A stale `STEER` target temporarily leads to bad vehicle positioning, whereas a missing or stale `BRAKE` command in a high-speed vehicle can result in catastrophic physical collisions or structural damage.
* **Fail-Safe Guarantee:** The system latches any `BRAKE > 0` state to override all subsequent `THROTTLE` inputs until an explicit `BRAKE 0` command is issued. Additionally, the Watchdog task periodically evaluates elapsed time using an atomic timestamp (`millis() - g_last_command_timestamp`). If no command arrives within **500 ms**, it forces the output to a distinct 5 Hz safe-idle blink pattern regardless of any stale queue items, guaranteeing deterministic fail-safe behavior instead of silently freezing.

### 3. What would you add or fix first if you had one more day?
1. **CRC Frame Verification:** Add CRC8 or CRC16 checksum checking to validate serial message integrity.
2. **Hardware Watchdog Integration:** Implement a Hardware Timer Watchdog (WDT) alongside the FreeRTOS task to trigger an MCU reset in the event of total task starvation or memory corruption.
3. **Mutex-Guarded FSM:** Formalize state transitions into a dedicated Finite State Machine class with strict lock protection around shared variables.

---

## Testing & Verification Script

Open Arduino IDE Serial Monitor (or any terminal emulator), set the baud rate to **115200**, and set the line ending to **Newline**.

| Step | Terminal Input | Expected Behavior |
|---|---|---|
| **1** | `PING` | Board responds immediately: `[ACK] PONG` |
| **2** | `THROTTLE 40` | Onboard LED ramps to ~40% brightness |
| **3** | `STEER -60` | Steering angle (-60) logged and displayed in STATUS telemetry |
| **4** | `THROTTLE 90` then `BRAKE 100` | LED snaps to 0% brightness immediately (BRAKE locks output to 0%) |
| **5** | `BRAKE 0` then `THROTTLE 55` | Output resumes normal operation at 55% PWM brightness |
| **6** | *(Do not type for > 600 ms)* | WATCHDOG trips: logs `"LINK LOST, failing safe"` and LED blinks at 5 Hz |
| **7** | `THROTTLE 20` | Fail-safe clears instantly; LED returns to steady 20% brightness |
| **8** | `THROTTLE abc` | Malformed line ignored safely without crashing, hanging, or resetting |
| **9** | `BRAKE 100` + `THROTTLE 100` (Burst) | `BRAKE` takes precedence; LED output remains locked at 0% |
https://drive.google.com/file/d/1B8sYpIWezv5ZPjIUfFPIgVqjd-ZbXu-G/view?usp=drive_link 
