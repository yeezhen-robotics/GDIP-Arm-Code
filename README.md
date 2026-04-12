# GDIP-Arm-Code

An Arduino-based robot arm implementation developed for the **GDIP third-year module at UWE**. The system is designed as a **medical vial transportation module**, featuring teach, autonomous, and homing operational paradigms. Communication between the custom handheld controller and the robot arm is handled wirelessly via RF antennas.

---

## Overview

The project spans two Arduino stations that communicate with each other wirelessly:

| Custom Controller      | Robot Arm              |
| ---------------------- | ---------------------- |
| Transmission.ino       | Gripper_Station.ino    |
|<img width="1300" height="2378" alt="image" src="https://github.com/user-attachments/assets/22a38463-9a9d-4213-818c-5caa582c1fb6" /> | ![20251219_113622](https://github.com/user-attachments/assets/0e014e74-c2ae-4a15-858a-cc1e0f089be4) |

---

## Repository Structure

```
GDIP-Arm-Code/
├── Gripper_Station/        # Arduino code for the robot arm & gripper
├── Transmission/           # Arduino code for the custom remote controller
├── Transmission_Test/      # Antenna troubleshooting — receiver test
└── Transmission_Test_True/ # Antenna troubleshooting — transmitter test
```

---

## Operational Paradigms

The robot arm supports three modes of operation:

- **Teach Mode** — the operator manually guides the arm to a target position, which is then recorded
- **Autonomous Mode** — the arm replays a saved sequence of positions to perform the transportation task automatically
- **Homing Mode** — returns the arm to its defined home/rest position

---

## Folders

### Gripper_Station

Arduino code to be flashed onto the **robot arm station**. Controls:
- Servo/stepper motor movements for each joint
- Gripper open/close actuation
- Receiving wireless commands from the controller
- Executing teach, autonomous, and homing routines

### Transmission

Arduino code to be flashed onto the **custom handheld remote controller**. Handles:
- Reading operator inputs (buttons, joysticks, switches)
- Encoding and transmitting commands wirelessly to the robot arm

### Transmission_Test

Troubleshooting code that configures the Arduino as a **receiver** to verify that the RF antenna is correctly picking up incoming signals. Use this when diagnosing antenna or communication issues on the receiving end.

### Transmission_Test_True

Troubleshooting code that configures the Arduino as a **transmitter** to verify that the RF antenna is correctly broadcasting signals. Use this in conjunction with `Transmission_Test` to confirm end-to-end wireless communication before deploying the main codebase.

---

## Hardware Requirements

- 2× Arduino boards (one per station)
- RF antenna modules (e.g. NRF24L01 or equivalent) for wireless communication
- Robot arm assembly with servo/stepper actuators
- Gripper mechanism
- Custom controller enclosure with input peripherals

---

## Flashing Instructions

1. Open the relevant `.ino` file in the **Arduino IDE**
2. Select the correct board and COM port under **Tools**
3. Click **Upload** to flash the code onto the Arduino

Flash `Transmission.ino` onto the controller Arduino and `Gripper_Station.ino` onto the arm Arduino.

> For first-time setup, use `Transmission_Test_True` (transmitter) and `Transmission_Test` (receiver) to confirm the wireless link is working before flashing the main code.

---

## License

MIT — see [LICENSE](LICENSE) for details.
