# Alphaflight

Alphaflight is an experimental flight controller firmware for fixed-wing aircraft,
built around a custom STM32F722-based flight controller.

The long-term goal of this project is fully autonomous fixed-wing flight,
including mission-based operation, automatic landings, and operation beyond
visual line of sight and transmitter range.

This is a personal research and learning project and is not intended to be a
general-purpose or production-ready flight controller.

## Hardware platform

Alphaflight targets a custom-designed flight controller PCB, designed in EasyEDA.

### On-board hardware
- **MCU:** STM32F722RET6
- **IMU:** BMI088  
  (planned migration to BMI270 with PCB v2)
- **Barometer:** BMP390
- **Storage:** 32 GB SD card

### External hardware
- **GPS:** TBS M10Q
- **RC link:** CRSF Nano RX Long Range (500 mW telemetry)
- **Motor control:** DSHOT ESC (2 outputs)
- **Actuators:** Standard PWM servos (4 outputs)

## Current focus

The current development focus is on:
- Basic hardware bring-up
- Sensor drivers (IMU, barometer, GPS)
- Reliable logging to SD card
- CRSF receive and telemetry integration
- Core flight control architecture

## Planned features

The following features are design goals and not necessarily implemented yet:

- SD card flight data logging
- CRSF RC decoding
- CRSF telemetry output
- GPS-based navigation
- Mission-based autonomous flights
- Automatic landing
- Operation beyond transmitter range using onboard autonomy
- Full fixed-wing autopilot
- Fly-by-wire control concept inspired by commercial aircraft (e.g. A320)

## Tooling

- **Language:** C
- **IDE:** STM32CubeMX, Visual Studio Code + STM32Cube
- **PCB design:** EasyEDA
- **Debugging:** ST-Link V2 (important!)

## Project status

Early development / bring-up phase.  
Architecture, hardware, and feature scope are subject to change.
