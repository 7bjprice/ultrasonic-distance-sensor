# Ultrasonic Distance Sensor

## Overview
This project is an embedded system designed as rear parking assist system that displays in real time the distance to an object with added saftey measures succh as audio and visual feedback. 
It was built using a STM microcontroller and integrates:
- An LCD display for output
- I²C protocol for communication with peripheral devices
- Timers and interrupts for precise event scheduling

## Features
- **Real-Time Data Display** – Updates values on the LCD in real time.
- **I²C Communication** – Efficient, two-wire communication with sensors or other peripherals.
- **Interrupt-Driven Timing** – Timers configured to handle periodic tasks without CPU busy-waiting.
- **Modular Code Structure** – Easily adaptable for other PIC-based projects.
- **Low-Power Considerations** – Designed with energy efficiency in mind.

## Hardware Requirements
- STM microcontroller
- LCD Character Display
- I²C-compatible sensors or devices  
- Breadboard 
- USB programmer/debugger
