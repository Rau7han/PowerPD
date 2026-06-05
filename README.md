# PowerPD - ESP32 USB-C PD/PPS Programmable Power Supply

![PowerPD Design](Picture/PowerPD%20Design.png)

## Introduction

PowerPD is an innovative ESP32-based USB-C Power Delivery (PD) and Power Profile Specification (PPS) programmable bench power supply. It leverages modern USB-C PD technology to provide a flexible, programmable power source that can deliver various voltage and current levels. Perfect for electronics enthusiasts, makers, and professionals who need a compact, feature-rich power supply for prototyping and testing.

## Specifications

- **Microcontroller**: ESP32
- **Input**: USB-C Power Delivery / PPS Charger
- **Output Voltage Range**: 5V to 20V (via USB PD/PPS protocols)
- **Programmable Voltage**: User-configurable voltage selection
- **Programmable Current**: User-adjustable current limits
- **Control Interface**: Interactive menu system via ESP32
- **Communication**: UART to USB interface for programming
- **Compact Design**: Space-efficient bench power supply form factor

## How It Works

1. **Power Input**: A USB-C PD/PPS charger supplies power to the system
2. **ESP32 Control**: The ESP32 microcontroller manages voltage negotiation and current limiting
3. **User Configuration**: Users set desired voltage and current values through an intuitive menu interface
4. **Power Output**: The regulated output is available as a programmable bench power supply, suitable for powering various electronics projects

## Repository Files

- **Firmware**: Contains the ESP32 Arduino code (`PowerPD.ino`), including the AP33772S power manager driver for voltage and current control
- **Gerber Files**: PCB manufacturing files ready for fabrication
- **Picture**: High-quality project photos and demonstration images
- **PowerPD Schematic**: Circuit schematics for reference and replication
- **STL**: 3D-printable enclosure files for project housing

## Project Images

![PowerPD 3D Model](Picture/PowePD%203D.png)

![PowerPD PCB](Picture/PowerPD%20pcb%20from%20nextpcb.jpg)

![5V Output](Picture/5V%20output.jpg)

![PPS Menu](Picture/PPS%20adjust%20Menu.jpg)

## Project Links

- [Hackster.io Project](https://www.hackster.io/rau7han/powerpd-esp32-usb-c-pd-pps-programmable-power-supply-c2c4c0)
- [Hackaday.io Project](https://hackaday.io/project/205791-powerpd-esp32-usb-c-pdpps-bench-power-supply)
- [Instructables Guide](https://www.instructables.com/PowerPD-ESP32-USB-C-PD-Bench-Power-Supply/)

---

**License**: See repository for license information  
**Author**: Rau7han
