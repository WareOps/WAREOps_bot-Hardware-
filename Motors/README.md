# `Motors` — Arduino Motor Firmware Package (`ROSArduinoBridge`)

**Deployment Target**: 🔵 **Local Computer** (Flashed to **Arduino Nano** via USB)

The `Motors` directory contains the Arduino C++ firmware sketch (`ROSArduinoBridge`) designed to run on the **Arduino Nano** microcontroller driving the physical robot's wheels.

Here I have used Mdd10A dual channel motor driver


---

## 📁 Package Structure

```text
Motors/
└── my_bridge/
    └── ROSArduinoBridge/
        ├── ROSArduinoBridge.ino  # Main Arduino sketch entry point & serial command parser
        ├── commands.h            # Serial command byte definitions ('e', 'm', 'r', 'p', 'o')
        ├── diff_controller.h     # Closed-loop PID controller calculation loop
        ├── encoder_driver.h      # Quadrature encoder pin definitions & macros
        ├── encoder_driver.ino    # Encoder interrupt handlers (Pin Change Interrupts)
        ├── motor_driver.h        # Cytron MDD10A motor driver pin mappings
        ├── motor_driver.ino      # PWM & DIR output logic
        ├── sensors.h             # Analog/digital sensor reading headers
        ├── servos.h              # Servo motor control header
        ├── servos.ino            # Servo PWM generation logic
        └── README.md             # Pinout and wiring documentation
```

---

## 🔌 Hardware & Wiring Specifications

### 1. Cytron MDD10A Dual Channel Motor Driver Wiring

| Signal | Arduino Nano Pin | MDD10A Pin | Function |
|---|---|---|---|
| **Left DIR** | `D7` | `M1DIR` | Left motor direction |
| **Left PWM** | `D10` | `M1PWM` | Left motor PWM speed |
| **Right DIR** | `D8` | `M2DIR` | Right motor direction |
| **Right PWM** | `D9` | `M2PWM` | Right motor PWM speed |
| **Enable (EN)** | TBD / +5V | `EN` | Driver enable (connect to +5V) |

### 2. TTN25 Quadrature Encoders Pin Mapping

| Motor | Channel A Pin | Channel B Pin |
|---|---|---|
| **Left Encoder** | `D2` | `D3` |
| **Right Encoder** | `A4` | `A5` |

---

## 💻 Serial Protocol Reference

Baud Rate: **57600 baud** (Carriage Return `\r` required).

| Command | Syntax | Description | Example |
|---|---|---|---|
| **Read Encoders** | `e` | Returns current encoder tick counts for both wheels | `e` -> `1420 1395` |
| **Reset Encoders** | `r` | Resets encoder counters to zero | `r` -> `OK` |
| **Open-Loop Speed** | `o <PWM1> <PWM2>` | Direct PWM drive (-255 to 255) | `o 100 -100` |
| **Closed-Loop Speed** | `m <Spd1> <Spd2>` | Closed-loop PID speed in encoder ticks per loop | `m 15 -15` |
| **Set PID Gains** | `p <Kp> <Kd> <Ki> <Ko>` | Updates PID loop parameters | `p 10 12 0 50` |

> ⚠️ **Safety Timeout**: If no speed commands (`m` or `o`) are received within **2.0 seconds**, the controller automatically stops both motors to prevent runaway conditions.

---

## ⚡ How to Flash Firmware

1. Install the **Arduino IDE** (or Arduino CLI) on your computer.
2. Open `Motors/my_bridge/ROSArduinoBridge/ROSArduinoBridge.ino`.
3. Select Board: **Arduino Nano** (Processor: ATmega328P or ATmega328P Old Bootloader).
4. Connect Arduino Nano via USB. Select the serial port (e.g. `/dev/ttyUSB0` or `/dev/ttyACM0`).
5. Click **Upload**.
6. Verify operation by opening the Serial Monitor at **57600 baud**, sending `e`, and checking for encoder tick responses.
