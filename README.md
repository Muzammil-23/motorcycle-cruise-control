# Low-Cost Cruise Control System for Honda CD70 (and similar 70-150cc motorcycles)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?logo=arduino)](https://store.arduino.cc/products/arduino-nano)
[![Platform](https://img.shields.io/badge/platform-Arduino%20AVR-blue)]()

A retrofittable, low-cost cruise control system for small-displacement motorcycles (70-150cc) commonly found in South Asian and Southeast Asian markets. The system uses an Arduino Nano, Hall-effect speed sensor, MG996R servo, and a discrete-time PI controller to maintain a rider-set speed, and 1.2 in OLED for UI, reducing fatigue on long rides.

## 📋 Features

- **Closed-loop speed regulation** at 50 km/h setpoint (configurable)
- **Velocity-form PI controller** (Kp = 0.67, Ki = 0.48 s⁻¹) with 2 km/h dead-band
- **Moving average filter** (3 samples) for noise reduction
- **Safety disengagement** via brake, clutch, and kill switch
- **Hardware interrupt** for brake → fastest response (~500 ms)
- **OLED display** showing speed, setpoint, and system status
- **Bumpless transfer** on engagement/disengagement
- **Cost-effective** (~$20 USD total hardware cost)

## 🔧 Hardware

This project was designed using **Proteus 8 Professional** .

| Folder | Contents |
|--------|----------|
| `/schematic` | Proteus editable file (.pdsprj) and PDF schematic |
| `/PCB` | Gerber files (ZIP) for manufacturing and PDF layout |

## 💻 Firmware

The firmware runs on an **Arduino Nano** (ATmega328P).

**Required Libraries:**
- `Adafruit_SSD1306.h` (OLED display)
- `Adafruit_GFX.h` (graphics library)
- `Servo.h` (built-in)

**Upload Instructions:**
1. Open the `.ino` file in Arduino IDE
2. Select Board: Arduino Nano
3. Select Processor: ATmega328P
4. Upload via USB

## 🖨️ 3D Models

STL files for the enclosure and mechanical parts are in the `/3D files` folder.

**Printing Recommendations:**
- Material: ABS or PETG
- Layer height: 0.2mm
- Infill: 20%
- Supports: Yes (for specific parts - check individual files)

# Project Licensing

This project uses different licenses for different types of content:

| Content Type | License |
|--------------|---------|
| Firmware (code in `/firmware` folder) | MIT License (see LICENSE file) |
| Hardware (schematics, PCB layouts, BOM) | CERN-OHL-P-2.0 (see LICENSE-Hardware file) |
| 3D Models (CAD files, STLs) | CERN-OHL-P-2.0 (see LICENSE-Hardware file) |

The MIT License applies ONLY to software files. The CERN-OHL-P-2.0 applies to all hardware design files.
