# Robocon 2025 — Robot Basketball
### Team Robotics | ABU Robocon 2025

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red)](https://www.raspberrypi.com/)
[![MCU](https://img.shields.io/badge/MCU-ESP32%20%7C%20Arduino%20Due-green)](https://www.espressif.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

---

## Overview

This repository contains the complete software stack developed for ABU Robocon 2025, themed **"Robot Basketball."** The challenge required robots capable of rapid omnidirectional movement, ball shooting, dribbling, and real-time defense.

Two robots were designed with distinct roles and distinct engineering approaches:

| | **R1 — Offense** | **R2 — Defense** |
|---|---|---|
| **Role** | Shooter + Dribbler | Rapid-response defender |
| **Drive** | 4-wheel omni | 3-wheel omni |
| **Brain** | Raspberry Pi 4 (ROS Noetic) | Arduino Due (bare-metal) |
| **Comms** | ROS topics → ESP32 → Due (USART) | ESP32 → Due (USART) |
| **Special** | BLDC shooter, pneumatic dribbler | Encoder-controlled sliding net |
| **Autonomy** | Active Heading Lock (IMU PID) | Active Heading Lock (IMU PID) |

---

## System Architecture

### R1 — Offense Robot

```
┌─────────────────────────────────────────────────────────┐
│                  Raspberry Pi 4 (ROS Noetic)             │
│                                                          │
│  [joystick_bridge.py]──►/ps4_data (Twist)               │
│         ▲                      │                         │
│  /joy   │          [imu_pipeline.py]──►/bno055_data      │
│  (joy)  │                      │                         │
│         │              [motion_controller.py]            │
│  [PS4]  │               ├── Field-centric transform      │
│         │               ├── IMU PID heading lock         │
│         │               ├── 3-wheel inverse kinematics   │
│         │               └──►/motor_value (Int32Array)    │
│                                      │                   │
│  [encoder_bridge.py]◄──USART──┐      │                   │
│         │                     │      ▼                   │
│  /encoder_distance             │  [motor_driver.py]      │
│         │                     │   ├── pigpio PWM × 3     │
│         ▼                  ESP32   ├── Pneumatic valve   │
│  [motion_controller.py]    (odometry │── BLDC ESC relay  │
│   encoder odometry)        + WebSocket)                  │
└─────────────────────────────────────────────────────────┘
                    │ USART (EasyTransfer, 115200)
                    ▼
         ┌─────────────────────┐
         │    Arduino Due       │
         │  Motor PWM × 3      │
         │  DIR pins × 3       │
         └─────────────────────┘
```

### R2 — Defense Robot

```
┌──────────────┐     USART (EasyTransfer)     ┌─────────────────────────┐
│   ESP32      │ ──────────────────────────► │     Arduino Due          │
│              │                              │                          │
│  PS4 BT      │   Struct: {x, y, buttons}   │  3-wheel omni IK         │
│  receiver    │                              │  IMU heading lock (PID)  │
│              │                              │  Sliding net controller  │
│              │                              │  Pneumatic valve driver  │
└──────────────┘                              └─────────────────────────┘
```

---

## Key Features

### 1. Active Heading Lock (Shared Autonomy)
Both robots implement **IMU-based closed-loop heading control** during teleoperation. The driver provides directional input; the system automatically maintains the desired heading using a PID controller reading from the BNO055 IMU. This eliminates rotational drift during fast omnidirectional movement and significantly reduces driver cognitive load during matches.

- **Sensor**: BNO055 9-DOF IMU (quaternion output)
- **Signal chain**: Quaternion → yaw → 5-sample median filter → exponential low-pass (α=0.1) → PID
- **Control loop rate**: 50 Hz
- **Gains**: Kp=3.0, Ki=0.05, Kd=0.0 (empirically tuned)

### 2. Field-Centric Drive
R1 implements **field-centric (world-frame) omnidirectional drive**. The driver's joystick commands are always relative to the field, not the robot's body. As the robot rotates, the IMU yaw is used to rotate the velocity vector into the robot's body frame before computing wheel speeds. The robot feels identical to drive regardless of its physical orientation.

### 3. FreeRTOS Dual-Core Odometry (R1 ESP32)
The ESP32 on R1 runs two **FreeRTOS tasks pinned to separate cores**:
- **Core 0**: 100 Hz quadrature encoder sampling from 3 encoders. Computes heading from differential X-encoders, integrates world-frame position, and streams odometry over USART.
- **Core 1**: Async WebSocket server for real-time pneumatic slider control via a phone/tablet UI.

### 4. Omni-Drive Inverse Kinematics (distinct per robot)

**R1 — 4-Wheel Omni IK**

Wheels are mounted at 45° to the chassis sides. Given body-frame `(vx, vy, ω)`:

```
ω₁ =  (1/2√2)(−vx + vy) + (L/R)·ω    # front-left
ω₂ =  (1/2√2)(−vx − vy) + (L/R)·ω    # front-right
ω₃ =  (1/2√2)( vx − vy) + (L/R)·ω    # rear-left
ω₄ =  (1/2√2)( vx + vy) + (L/R)·ω    # rear-right
```

where L/R = robot half-diagonal / wheel radius = 0.09 / 0.075 = 1.2

**R2 — 3-Wheel Omni IK**

Symmetric 120° wheel layout. Given body-frame `(vx, vy, ω)`:

```
ω₁ = (−2vx / 3)                + (ω / 3)    # front
ω₂ = (  vx / 3) + (vy / √3)   + (ω / 3)    # rear-right
ω₃ = (  vx / 3) − (vy / √3)   + (ω / 3)    # rear-left
```

In both cases, speeds are proportionally scaled to the PWM range while preserving the velocity direction ratio.

### 5. Multi-Actuator Upper Control (R1)
A single ESP32 manages three independent actuators on R1's upper mechanism:
- **BLDC motor** (ball shooter): ESC controlled via 50 Hz PWM (1000–2000 µs). Boot-time calibration sequence arms the ESC safely.
- **Stepper motor** (shooter angle): encoder-verified position control with a gear ratio of 8800 ticks → 23°. Closed-loop angle correction to ±0.2° resolution.
- **4-channel pneumatics** (dribbler): a timed state-machine sequence (extend → hold → retract) triggered by driver input.

### 6. Encoder-Controlled Sliding Net (R2)
R2's net mechanism uses a **encoder-gated DC motor** to deploy and retract a physical net for defense. A rising-edge detector on the Triangle button triggers motor movement; the motor runs until a calibrated encoder tick threshold (`targetTicks = 700`) is reached, then stops automatically — preventing mechanical over-travel without limit switches.

---

## Repository Structure

```
robocon-2025/
│
├── r1_offense/
│   ├── ros_ws/                        # ROS Noetic workspace (Raspberry Pi 4)
│   │   ├── motion_controller.py       # Core: field-centric drive + IMU PID
│   │   ├── imu_pipeline.py            # BNO055 I2C driver + dual filter chain
│   │   ├── motor_driver.py            # pigpio PWM bridge + actuator control
│   │   ├── joystick_bridge.py         # PS4 DualShock → ROS Twist
│   │   └── encoder_bridge.py          # USART → ROS encoder odometry
│   │
│   └── firmware/
│       ├── esp32_odometry_rtos/
│       │   └── esp32_odometry_rtos.ino  # FreeRTOS dual-core odometry + WebSocket
│       └── upper_control/
│           └── upper_control.ino        # BLDC + stepper + pneumatics (ESP32)
│
└── r2_defense/
    └── firmware/
        └── due_omni_controller/
            └── due_omni_controller.ino  # Full defense bot: omni IK + net + pneumatics
```

---

## Hardware Summary

### R1 — Offense
| Component | Part | Interface |
|---|---|---|
| Compute | Raspberry Pi 4 (4GB) | — |
| Odometry MCU | ESP32 DevKit | USART to Due, WiFi WebSocket |
| Motor MCU | Arduino Due | USART from Pi, PWM to drivers |
| IMU | BNO055 | I2C to RPi |
| Drive motors | 4× Brushed DC + encoders (omni) | pigpio PWM |
| Shooter | BLDC motor + ESC | PWM (50 Hz, 1000–2000 µs) |
| Shooter angle | Stepper + quadrature encoder | STEP/DIR pulses |
| Dribbler | Pneumatic cylinder × 2 (4 valves) | GPIO |
| Controller | PS4 DualShock 4 | Bluetooth → `joy` node |

### R2 — Defense
| Component | Part | Interface |
|---|---|---|
| MCU | Arduino Due | — |
| Wireless | ESP32 DevKit | USART EasyTransfer |
| IMU | ADXRS (gyro) | SPI |
| Drive motors | 3× Brushed DC | Due PWM |
| Net mechanism | DC motor + quadrature encoder | Due PWM + digital |
| Pneumatics | 2-valve solenoid | GPIO (pins 52, 53) |
| Controller | PS4 DualShock 4 | Bluetooth → ESP32 |

---

## Software Dependencies

### ROS Workspace (Python 3, ROS Noetic)
```
rospy
sensor_msgs
geometry_msgs
std_msgs
pigpio          # GPIO/PWM on RPi
adafruit-circuitpython-bno055
board, busio    # I2C on RPi
pyserial        # USART encoder bridge
```

### Arduino / ESP32 Firmware
```
EasyTransfer    # Structured USART comms
ESP32Encoder    # Quadrature encoder library
ESP32Servo      # Servo/ESC PWM on ESP32
AsyncTCP        # Async TCP for WebSocket
ESPAsyncWebServer
ArduinoJson
```

---

## Running the System (R1)

```bash
# 1. Launch IMU publisher
rosrun robocon_node imu_pipeline.py

# 2. Launch joystick driver
roslaunch joy joy.launch

# 3. Launch PS4 bridge
rosrun robocon_node joystick_bridge.py

# 4. Launch encoder USART bridge
rosrun robocon_node encoder_bridge.py

# 5. Launch motor driver
rosrun robocon_node motor_driver.py

# 6. Launch main motion controller (heading lock + IK)
rosrun robocon_node motion_controller.py
```

---

## What I Built

This project was my primary engineering contribution across a full competitive robotics season:

- Designed and implemented the **field-centric drive + IMU heading lock** system from scratch, including the dual-filter IMU signal chain and PID tuning
- Wrote the **FreeRTOS dual-core ESP32 firmware** for concurrent encoder odometry and WebSocket control
- Implemented **3-wheel and 4-wheel omni inverse kinematics** on both ROS (Python) and bare-metal Arduino (C++)
- Developed the **multi-actuator upper control system** coordinating BLDC, stepper, and pneumatics on a single ESP32
- Built the **USART EasyTransfer communication bridge** linking the ROS compute layer to the lower-level microcontrollers
- Designed the **encoder-gated sliding net mechanism** for R2's defense system
- Wrote the complete **BNO055 IMU pipeline** including quaternion-to-yaw conversion, median filter, and exponential smoothing

---

## License

MIT License. See [LICENSE](LICENSE).
