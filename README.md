# Morse Code Emulator - VirtLab

## Overview
This project implements a Morse code emulator using the **VirtLab** board provided by Politecnico di Torino for the **Embedded Systems Integration** course. The firmware allows users to input words through a **PuTTY terminal**, which are then converted to Morse code and displayed both on the terminal and through the LEDs on the VirtLab board.

## Features
- **User Input via PuTTY**: Users can type words into the serial terminal.
- **Morse Code Conversion**: The input is translated into Morse code.
- **Visual Feedback**: The Morse code is displayed on the terminal.
- **LED Output**: The VirtLab board's LEDs blink according to the Morse code timing.

## Hardware & Software Requirements
### **Hardware**
- **VirtLab Board** (provided by Politecnico di Torino)
- **MCU: STM32L496VETx** (by STMicroelectronics)
- USB connection for serial communication
- LEDs integrated into the VirtLab board

### **Software**
- **STM32CubeIDE** (for firmware development and debugging)
- **FreeRTOS** (for real-time task scheduling)
- **PuTTY** or any serial terminal emulator
- **Embedded C Compiler** (integrated in STM32CubeIDE)
- VirtLab SDK (if required for flashing firmware)

## How It Works
1. The user types a word in the **PuTTY** terminal.
2. The firmware processes the input and converts each character into Morse code.
3. The Morse code is:
   - Displayed in text format on the terminal.
   - Transmitted using the **VirtLab board’s LEDs** (short blinks for dots, long blinks for dashes).
4. The firmware is implemented using **FreeRTOS**, allowing concurrent task management for handling user input, Morse code conversion, and LED signaling efficiently.

## Installation & Usage
1. **Flash the Firmware**
   - Develop and compile the firmware using **STM32CubeIDE**.
   - Upload the compiled binary to the **VirtLab board**.

2. **Connect to PuTTY**
   - Open **PuTTY** and configure the correct **serial port**.
   - Set **baud rate**, **parity**, and **stop bits** according to VirtLab specifications.

3. **Run the Program**
   - Type a word into the terminal.
   - Observe the Morse code translation on the terminal and LED output.

## Example Execution
```
Enter a word: HELLO
Morse Code: .... . .-.. .-.. ---
```
(LEDs blink according to the Morse code sequence.)

## Future Enhancements
- Add support for real-time audio output (buzzer integration for Morse beeps).
- Implement error handling for invalid inputs.
- Expand to support full ASCII character conversion.

This project demonstrates an interactive way to learn Morse code using embedded systems!

