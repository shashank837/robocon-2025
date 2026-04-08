# Robocon 2025 — Robot Basketball

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![MCU](https://img.shields.io/badge/MCU-ESP32%20%7C%20Arduino%20Due-green)](https://www.espressif.com/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red)](https://www.raspberrypi.com/)

This is the software I wrote for our ABU Robocon 2025 team. The theme was **Robot Basketball** — two robots had to move a ball around a court, shoot it into a basket, and block the other team from scoring.

We built two robots with completely different roles, which ended up using pretty different tech stacks too.

---

## The Two Robots

| | R1 — Offense | R2 — Defense |
|---|---|---|
| **Job** | Dribble and shoot the ball | Block the other robot |
| **Drive** | 4-wheel omni | 3-wheel omni |
| **Main compute** | Raspberry Pi 4 (ROS Noetic) | Arduino Due (bare-metal) |
| **Comms** | RPi → ESP32 → Due via USART | ESP32 → Due via USART |
| **Special mechanisms** | BLDC shooter + pneumatic dribbler | Encoder-controlled sliding net |
| **Autonomy feature** | Active Heading Lock (IMU + PID) | Active Heading Lock (IMU + PID) |

---

## System Architecture

### R1 — Offense Robot

R1 runs ROS Noetic on a Raspberry Pi 4. All the high-level logic (field-centric drive, heading lock PID, inverse kinematics) lives in Python ROS nodes. The Pi talks to an ESP32 over USART, which handles encoder odometry and streams position data back.

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                     Raspberry Pi 4  (ROS Noetic)                    │
  │                                                                     │
  │  [PS4 Controller]                                                   │
  │       │ Bluetooth → joy node → /joy                                 │
  │       ▼                                                             │
  │  [joystick_bridge.py] ──► /ps4_data (Twist: vx, vy, heading_target)│
  │                                   │                                 │
  │  [imu_pipeline.py]                │                                 │
  │   BNO055 over I2C                 │                                 │
  │   quaternion → yaw                │                                 │
  │   median filter + LPF             │                                 │
  │       │ /bno055_data (Float32)    │                                 │
  │       └───────────────────────────▼                                 │
  │                          [motion_controller.py]   (50 Hz loop)      │
  │                           ├─ field-centric velocity transform       │
  │                           ├─ heading lock PID  →  omega             │
  │                           ├─ 4-wheel omni inverse kinematics        │
  │                           └──► /motor_value (Int32MultiArray x4)    │
  │                                         │                           │
  │  [encoder_bridge.py]                    ▼                           │
  │   USART ← ESP32               [motor_driver.py]                     │
  │   /encoder_distance ──────►    ├─ pigpio PWM × 4  (drive motors)   │
  │   (feeds odometry back         ├─ pneumatic dribbler valves (GPIO)  │
  │    into motion_controller)     └─ BLDC shooter relay (GPIO)         │
  │                                         │ pigpio / GPIO             │
  └─────────────────────────────────────────┼─────────────────────────-─┘
                                            │
           ┌────────────────────────────────┘
           │
  ┌──────────────────────────────────────────────────────────┐
  │  ESP32  (FreeRTOS)                                       │
  │                                                          │
  │  Core 0 — Task_EncoderRead  @ 100 Hz                    │
  │   Reads 3 quadrature encoders (Xu, Xd, Y)               │
  │   Heading from differential X encoders                   │
  │   Integrates world-frame X, Y position                   │
  │   Streams odometry → RPi over USART (EasyTransfer)      │
  └──────────────────────────────────────────────────────────┘
```

### R2 — Defense Robot

R2 runs entirely on an Arduino Due — no OS, no ROS. An ESP32 handles PS4 Bluetooth and forwards the controller state over USART. Everything else (IK, heading lock, net control) is one bare-metal loop on the Due.

```
  ┌───────────────────┐                    ┌────────────────────────────────────┐
  │      ESP32        │                    │            Arduino Due              │
  │                   │  USART             │                                    │
  │  PS4 Bluetooth    │  EasyTransfer      │  3-wheel omni inverse kinematics   │
  │  receiver         │ ─────────────────► │  Active heading lock (P control)   │
  │                   │                    │  Encoder-gated net deployment       │
  │                   │  struct {          │  Pneumatic valve driver             │
  │                   │   x, y,            │                                    │
  │                   │   buttons,         │  Motor PWM × 3  +  DIR × 3         │
  │                   │   angle }          │  Net motor PWM  +  encoder          │
  └───────────────────┘                    └────────────────────────────────────┘
```

---

## Key Features

### Active Heading Lock

This was probably the most useful thing we built. Both robots are teleoperated, but instead of the driver constantly correcting for drift, the IMU automatically keeps the robot pointing in the right direction while you drive anywhere.

The driver's right stick sets a heading target. A PID loop running at 50 Hz computes how much angular correction (`ω`) to inject into the wheel speeds. The driver only has to think about where to go, not which way the robot is facing.

On R1 the IMU is a BNO055 connected over I2C to the Pi. Raw quaternion output is converted to yaw, run through a 5-sample median filter to kill I2C glitch spikes, then through an exponential low-pass filter (α = 0.1) to smooth it before it enters the PID:

```
error  = target_heading − current_yaw     (wrapped to ±180°)
ω      = Kp·error + Ki·∫error dt         (Kp = 3.0, Ki = 0.05)
```

On R2 the same idea runs on the Due using the gyro angle that arrives in the EasyTransfer struct.

### Field-Centric Drive (R1)

The driver's left stick always means the same direction on the field regardless of which way R1 is physically pointing. Forward = forward on the field, always.

Before computing wheel speeds we rotate the joystick velocity vector from world frame into the robot's current body frame using the IMU yaw:

```python
vx_body =  vx_world * cos(yaw) + vy_world * sin(yaw)
vy_body = -vx_world * sin(yaw) + vy_world * cos(yaw)
```

Then `(vx_body, vy_body, ω)` goes into the IK. Makes a huge difference in how easy the robot is to drive under pressure.

### Inverse Kinematics

**R1 — 4-wheel omni** (wheels mounted at 45° to chassis sides):

```
K    = 1 / (2√2) ≈ 0.3536
L_R  = robot_half_diagonal / wheel_radius = 0.09 / 0.075 = 1.2

ω₁ = K·(−vx + vy) + L_R·ω    ← front-left
ω₂ = K·(−vx − vy) + L_R·ω    ← front-right
ω₃ = K·( vx − vy) + L_R·ω    ← rear-left
ω₄ = K·( vx + vy) + L_R·ω    ← rear-right
```

**R2 — 3-wheel omni** (120° symmetric layout):

```
ω₁ = (−2vx / 3)               + (ω / 3)    ← front
ω₂ = ( vx / 3) + (vy / √3)   + (ω / 3)    ← rear-right
ω₃ = ( vx / 3) − (vy / √3)   + (ω / 3)    ← rear-left
```

In both cases the outputs are proportionally scaled so the fastest wheel hits PWM 255 and the others scale down, keeping the velocity direction ratio intact.

### FreeRTOS Encoder Odometry (R1 ESP32)

The ESP32 on R1 runs a FreeRTOS task pinned to Core 0 that reads three quadrature encoders at 100 Hz. It uses the tick difference between the upper and lower X-axis encoders to estimate heading (`Δangle = (ΔxD − ΔxU) × dist_per_pulse / (2 × enc_radius)`), then integrates world-frame X/Y position. The result is streamed to the RPi over USART via EasyTransfer.

We used FreeRTOS here mainly to keep the encoder sampling on a dedicated core at a fixed rate — missing encoder pulses means wrong odometry, so it can't be blocked by anything else running on the same core.

### Sliding Net Mechanism (R2)

R2 has a physical sliding net that deploys to block the ball. It's driven by a DC motor with a quadrature encoder. Instead of using a limit switch we count encoder ticks from the moment the driver presses Triangle and stop the motor after 700 ticks — enough to fully deploy without over-travel. Rising-edge detection (`triangle && !trianglePrevState`) prevents re-triggering on a held button.

### Multi-Actuator Upper Control (R1)

A single ESP32 manages three completely independent actuators on R1's upper mechanism through serial commands:

- **BLDC shooter** via an ESC — PWM at 50 Hz (1000–2000 µs). ESC calibration sequence runs in `setup()` so it's armed before match start. Command: `T <throttle_us>`
- **Stepper motor** for shooter angle — step pulses with encoder feedback to verify final angle. Gear ratio: 8800 encoder ticks = 23°. Command: `A <degrees>`
- **Pneumatic dribbler** — 4 solenoid valves in a timed extend → hold → retract sequence. Command: `D`

---

## Repo Structure

```
robocon-2025/
├── README.md
│
├── r1_offense/
│   ├── ros_ws/
│   │   ├── motion_controller.py    # main loop: field-centric + heading PID + 4-wheel IK
│   │   ├── imu_pipeline.py         # BNO055 I2C → quaternion → yaw → median+LPF → publish
│   │   ├── motor_driver.py         # /motor_value → pigpio PWM ×4, + pneumatics + BLDC relay
│   │   ├── joystick_bridge.py      # /joy (Joy) → /ps4_data (Twist)
│   │   └── encoder_bridge.py       # USART serial → /encoder_distance (Float32MultiArray)
│   │
│   └── firmware/
│       ├── esp32_odometry_rtos/
│       │   └── esp32_odometry_rtos.ino   # FreeRTOS encoder odometry → USART to RPi
│       └── upper_control/
│           └── upper_control.ino         # BLDC ESC + stepper angle control + pneumatic dribbler
│
└── r2_defense/
    └── firmware/
        └── due_omni_controller/
            └── due_omni_controller.ino   # 3-wheel IK + heading lock + net mechanism + pneumatics
```

---

## Hardware

### R1 — Offense

| Component | Details |
|---|---|
| Compute | Raspberry Pi 4 (4 GB) |
| IMU | Bosch BNO055 — I2C, NDOF quaternion fusion mode |
| Odometry MCU | ESP32 DevKit — FreeRTOS, 3× quadrature encoders |
| Motor MCU | Arduino Due — receives /motor_value over USART |
| Drive | 4× brushed DC motors, omni wheels |
| Shooter | BLDC motor + ESC, 50 Hz PWM (1000–2000 µs) |
| Aiming | Stepper motor + quadrature encoder (8800 ticks = 23°) |
| Dribbler | 4-channel pneumatic solenoid valves (2 cylinders) |
| Controller | PS4 DualShock 4, Bluetooth |

### R2 — Defense

| Component | Details |
|---|---|
| MCU | Arduino Due |
| Wireless MCU | ESP32 DevKit — PS4 Bluetooth → USART EasyTransfer |
| IMU | ADXRS gyroscope — SPI |
| Drive | 3× brushed DC motors, omni wheels |
| Net | DC motor + quadrature encoder (700-tick travel limit) |
| Pneumatics | 2-valve solenoid (pins 52, 53) |
| Controller | PS4 DualShock 4, Bluetooth |

---

## Dependencies

### ROS Nodes (Python 3, ROS Noetic)
```
rospy  sensor_msgs  geometry_msgs  std_msgs
pigpio
adafruit-circuitpython-bno055
pyserial
```

### Arduino / ESP32 Firmware
```
EasyTransfer        # structured USART binary protocol
ESP32Encoder        # quadrature decoding on ESP32
ESP32Servo          # ESC PWM on ESP32
AsyncTCP + ESPAsyncWebServer
ArduinoJson
PS4Controller
```

---

## Running R1

```bash
# Flash esp32_odometry_rtos.ino to the ESP32 (update WiFi credentials first)
# Then on the Pi:

rosrun robocon_node imu_pipeline.py       # terminal 1
roslaunch joy joy.launch                   # terminal 2
rosrun robocon_node joystick_bridge.py    # terminal 3
rosrun robocon_node encoder_bridge.py     # terminal 4
rosrun robocon_node motor_driver.py       # terminal 5
rosrun robocon_node motion_controller.py  # terminal 6
```

For R2, just flash `due_omni_controller.ino` to the Due. No other setup needed.

---

## What I Worked On

- Full **ROS software stack for R1** — all five Python nodes
- **Active heading lock** — PID design, IMU signal chain, tuning
- **Field-centric drive** — IMU yaw coordinate transform
- **4-wheel and 3-wheel omni inverse kinematics** in both Python (ROS) and C++ (bare-metal)
- **FreeRTOS ESP32 firmware** — encoder odometry on a dedicated core, streamed to RPi over USART
- **Upper control firmware** — coordinating BLDC, stepper, and pneumatics on one ESP32
- **USART EasyTransfer communication layer** across all microcontrollers
- **R2 defense controller** — full bare-metal firmware on Arduino Due including the encoder-gated net mechanism

---

*ABU Robocon 2025 — India National Round*
