# Real-Time Embedded Application using FreeRTOS and PWM Control

This project implements a **real-time embedded application** using **FreeRTOS** on a **dsPIC microcontroller**. The system controls a PWM-driven actuator (e.g., a servo motor) based on temperature readings or manual input. It includes task scheduling, sensor acquisition, LCD display, and serial communication.

## 🎯 Project Objective

- Read temperature from a **DS18S20 digital sensor**
- Control a **PWM-based actuator (trapdoor/servo)**
- Switch between **automatic (temperature-based)** and **manual (ADC-based)** control modes
- Display system state on **LCD**
- Receive user commands via **UART serial interface**
- Structure the application using **FreeRTOS tasks** for real-time multitasking

## 🧱 Architecture & Components

### 🔧 Hardware

- **dsPIC microcontroller** on a development board
- **DS18S20 / DS18B20** temperature sensor
- **PWM output** to control a servo motor (via `P1DC3`)
- **Analog input (RB3)** for manual mode via potentiometer
- **LCD display (4 lines)**
- **UART** for PC communication (menu commands)
- **Push button (SW1)** for interrupt-based mode switching

### ⚙️ Software Structure

The main application is divided into **5 FreeRTOS tasks**:

| Task       | Functionality                                | Priority |
|------------|-----------------------------------------------|----------|
| `T1`       | Global state controller, handles SW1 INT      | +5       |
| `T2`       | Reads temperature from DS18S20                | +2       |
| `T3`       | Updates LCD display                           | +2       |
| `T4`       | Controls PWM output based on temperature/ADC  | +4       |
| `T5`       | Handles serial communication (menu & commands)| +3       |

### 🖥️ Display Information

The LCD shows:
- **Current temperature**
- **Control mode**: Automatic / Manual
- **Voltage read via ADC**
- **Last received serial command**

### 🔌 Serial Communication (UART)

The device listens for commands from the PC via serial terminal. The interface provides:
- `1`: Show current mode
- `2`: Toggle control mode
- `3`: Query current temperature

The menu is automatically printed after every command.

## 📐 Control Logic

- **Automatic Mode**:  
  - Temperature → PWM Duty Cycle (`P1DC3`)
  - LED ON (RB1)
- **Manual Mode**:  
  - ADC (RB3) → PWM Duty Cycle
  - LED OFF (RB1)
- Button `SW1` toggles the application state via external interrupt

## 📁 Source File Highlights

- `main.c` – FreeRTOS task definitions & scheduling
- `ds18s20.c/h` – Temperature sensor driver
- `new_lcd.c/h` – LCD driver
- `new_serial.c/h` – UART serial driver
- `pwm.c` – PWM control routines

## 🧪 Testing & Behavior

- All tasks are managed concurrently by FreeRTOS
- Task priorities ensure real-time responsiveness
- PWM reacts live to environmental input or manual voltage
- Commands from PC are handled immediately via UART

## 👤 Authors

- **Ștefan Atomulesei**  
- **Alexandru-Șerban Guță**  
Faculty of Automation and Computer Science  
Technical University of Iași, 2025
