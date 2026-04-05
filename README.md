# EdgeRover 🤖

A 4-wheeled robot with onboard edge AI — Bluetooth RC in V1, autonomous obstacle avoidance and person tracking via ESP32-CAM in V2.

> **Current Status:** V1 complete. V2 in progress.

---

## Overview

EdgeRover is a ground-up hardware + firmware project built in two phases.

**V1 (complete):** A Bluetooth-controlled RC car where an ESP32 receives D-pad input from a gamepad via Bluepad32 and drives four DC motors through an L298N motor driver using a differential (tank) steering model.

**V2 (in progress):** The remote control gets replaced entirely. An ESP32-CAM module mounted on the bot will run lightweight computer vision models directly on the microcontroller — no external PC, no cloud, no phone. The goal is a robot that can navigate its environment and track a person autonomously, powered entirely by edge inference on a $10 module.

**Why this matters:** Running CV inference on a microcontroller is a real constraint problem. The ESP32-CAM has 520KB of SRAM and no GPU. Every model decision — architecture, quantization, resolution — has to be made with that in mind. That's what makes V2 an interesting engineering challenge, not just a wiring project.

---

## Roadmap

### ✅ V1 — Bluetooth RC Car (Complete)
- ESP32 + Bluepad32 receiving gamepad input over Bluetooth
- L298N motor driver controlling 4 DC motors
- Tank steering (differential drive) — no servo required
- Fully wireless and battery powered

### 🔧 V2 — Autonomous Navigation (In Progress)
- ESP32-CAM module mounted on chassis for onboard vision
- **Obstacle avoidance** — detect and react to objects in the path in real time
- **Person tracking** — identify and follow a person using a lightweight detection model
- Both behaviors running on-device (no cloud, no external compute)
- Exploring TinyML / model quantization to fit inference within ESP32-CAM constraints

---

## Features (V1)

- 🎮 Bluetooth gamepad control via Bluepad32
- ⬆️ D-pad based movement — forward, backward, left, right
- ⚙️ Tank steering (differential drive) — pivot turns, no servo needed
- 🔋 Fully wireless and battery-powered

---

## Hardware Components

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 (38-pin DevKit) |
| Camera Module *(V2)* | ESP32-CAM (OV2640) |
| Motor Driver | L298N Dual H-Bridge |
| Motors | 4× TT DC Gear Motors (3–6V) |
| Chassis | 4WD Robot Car Chassis Kit |
| Power | 7.4V Li-ion battery pack (2S) |
| Controller *(V1)* | Any Bluepad32-compatible Bluetooth gamepad |
| Misc | Jumper wires, breadboard, screws, standoffs |

---

## Working Principle

**V1:**
```
Gamepad (Bluetooth) → ESP32 (Bluepad32) → L298N Motor Driver → DC Motors
```

**V2 (planned):**
```
ESP32-CAM (capture frame) → CV model inference (on-device) → Motor commands → L298N → DC Motors
```

In V2, the control loop is fully closed on the robot itself. The camera captures a frame, the model runs inference to detect obstacles or a person, and the output directly maps to motor commands — all within the ESP32-CAM.

---

## Drive Architecture

EdgeRover uses a **differential drive (tank steering)** model:

- The 4 motors are split into **two independent groups** — left side and right side
- All left motors are wired together and treated as a single unit
- All right motors are wired together and treated as a single unit
- Turning is achieved by driving one side forward and the other backward
- The bot pivots on the spot — no steering servo required

This is the same steering model used in tracked vehicles, combat robots, and most autonomous ground robots. It also simplifies V2: the autonomous controller only needs to output left-speed and right-speed to steer, exactly like the gamepad does in V1.

---

## Control Logic (V1)

| D-pad Input | Left Motors | Right Motors | Result |
|-------------|-------------|--------------|--------|
| Up | Forward | Forward | Move forward |
| Down | Backward | Backward | Move backward |
| Left | Backward | Forward | Pivot left |
| Right | Forward | Backward | Pivot right |
| None | Stop | Stop | Idle |

---

## Pin Connections

| ESP32 GPIO | L298N Pin | Function |
|------------|-----------|----------|
| GPIO 27 | IN1 | Left motors – direction A |
| GPIO 26 | IN2 | Left motors – direction B |
| GPIO 25 | IN3 | Right motors – direction A |
| GPIO 33 | IN4 | Right motors – direction B |
| GND | GND | Common ground |

> ENA and ENB on the L298N are jumpered HIGH (full speed). PWM speed control is planned for V2 to allow smoother autonomous movement.

---

## V2 — Edge AI Design Notes

The core challenge of V2 is fitting real-time computer vision onto a microcontroller with severe hardware constraints:

| Resource | ESP32-CAM |
|----------|-----------|
| CPU | Xtensa LX6, 240MHz |
| SRAM | 520KB (+ 4MB PSRAM on some boards) |
| Flash | 4MB |
| Camera | OV2640, up to 1600×1200 (inference will use much lower res) |

**Planned approach:**
- Use a heavily quantized model (INT8) to reduce memory footprint
- Run inference at low resolution (e.g. 96×96 or 160×120) to stay within SRAM limits
- **Obstacle avoidance:** object detection to identify blockers and route around them
- **Person tracking:** lightweight person detector (exploring MobileNet-based or custom trained TFLite model) to locate a person in frame and steer toward them
- Frameworks under consideration: TensorFlow Lite for Microcontrollers, ESP-DL

This is an active area of exploration — V2 design decisions will be documented here as they are made.

---

## Images / Demo

> 📸 *Bot photo*
> `![EdgeRover V1](assets/images/car.jpg)`

> 🔌 *Wiring diagram*
> `![Wiring Diagram](assets/diagrams/wiring.png)`

> 🎥 *Demo video (optional)*

---

## Challenges Faced (V1)

**Motor mismatch:** Motors didn't run at identical speeds even with the same signal, causing the bot to drift. Fixed by physically testing and pairing motors by speed.

**Wiring at scale:** 4 motors + motor driver + ESP32 = a lot of connections. One loose ground caused random motor cutouts that looked like a firmware bug for hours.

**Learning to solder:** First time soldering motor leads. Cold joints on two motors caused intermittent failures. Had to identify and reflow them.

**Debugging movement:** Figuring out which IN pin controlled which motor required systematically toggling each GPIO and tracing wires physically. Nothing was labeled.

**Power separation:** Powering the ESP32 and motors from the same rail caused the ESP32 to reset under motor load. Fixed by using the L298N's onboard 5V regulator to power the ESP32 separately.

---

## What I Learned

| Area | Takeaway |
|------|----------|
| Soldering | Joint quality directly affects reliability — cold joints are silent killers |
| Motor control | H-bridge logic, IN1/IN2 direction combos, power rail isolation |
| Embedded systems | GPIO control and digital output on ESP32 via Arduino framework |
| Bluetooth | How Bluepad32 abstracts HID gamepad input at the firmware level |
| Hardware debugging | Distinguishing software bugs from wiring faults from power issues |
| Drive systems | Differential drive theory and why it's the right choice for a 4WD robot |
| Edge AI *(upcoming)* | Model quantization, TFLite for microcontrollers, inference under memory constraints |

---

## Project Structure

```
EdgeRover/
├── README.md
├── src/
│   └── main.ino          # V1 Bluetooth RC firmware
└── assets/
    ├── images/
    │   └── car.jpg
    └── diagrams/
        └── wiring.png
```

---

## Getting Started (V1)

**Dependencies:**
- [Arduino IDE](https://www.arduino.cc/en/software) or PlatformIO
- [Bluepad32 library](https://github.com/ricardoquesada/bluepad32) — install via Arduino Library Manager

**Steps:**
1. Install Bluepad32 via Arduino Library Manager
2. Select your ESP32 board in Arduino IDE
3. Flash `src/main.ino` to the ESP32
4. Power on the bot and pair your gamepad
5. Drive

---

## License

MIT — free to use, modify, and build on.

---

*Building toward autonomous edge robotics, one phase at a time.*
