# nPZero nRF52 Host using Zephyr
![](/Documentation/nP0.png "nP0 nRF52")<br>

This repository provides demonstrations on how to use the **nPZero Driver API** with the nRF52 microcontroller platform running Zephyr RTOS, focusing on setting up code examples for peripherals and the ADC channels.

## API Driver Overview
The **nPZero Driver** provides a comprehensive API for interfacing with the nPZero IC. It can be used on any hardware platform to interact with the IC, offering a register-level interface that allows users to efficiently control and manage power-related functionalities in their applications.
The driver also includes a Hardware Abstraction Layer (HAL) for I2C operations, which can be customized to match specific hardware configurations, making it adaptable across various platforms.

## Executing Code Examples with the nPZero Driver
This project utilizes the **nPZero Driver** to execute the following functionalities:

- The Peripheral Mode examples demonstrate communication using I2C:
    - These examples show how to communicate with the temperature sensor connected via the I2C interface, which is bundled with the
      nPZero G1 Development Kit (DevKit).

- The ADC examples illustrates how to configure the internal and external ADC channels.

These examples demonstrate the necessary procedures for initialization, configuration, and data exchange required for sensor interaction using the **nPZero Driver**.

## Hardware Requirements
This project is designed to be executed on the nRF52 Development kits (DKs). In order to run, flash and debug the code you will need a debugger such as the Segger J-LINK or the ST-Link, connected to the Debug in connector (P18) on the nRF52DK.

In addition to the nRF52DK, the following PCBs, included in the nPZero G1 Development Kit (DevKit), are required:
- nPZero G1 Development Board (DevBoard)
- Temperature sensor AS6212

## Software Requirements
- nRF Connect SDK v3.0.0 (installation instructions [here](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/install_ncs.html))

## Setting Up and Developing for the nRF52 MCU Host
This section outlines the essential steps to develop, program, and debug an nRF52 MCU host using the nRF Connect Extension in Visual Studio Code and to set up UART
communication for logging.

### Hardware Setup Instructions
Follow these steps to set up the hardware:

1. Choose one of the following options to connect the nRF52 DK to the nPZero DevKit:
    - Arduino connectors (J5A, J6A, J7A, and J8A)
    - PMOD MCU connector (J17), which allows for the manual connection of any compatible MCU or radio chip
2. Attach the J-LINK debugger to the Debug In connector (P18) on the nRF52 DK
3. Connect the FTDI cable to the onboard UART pins on the nRF52 DK. The GPIO pins used are:
    - P0.06 (TXD)
    - P0.08 (RXD)
    - GND
    - VDD

   The UART signals are routed directly to the interface MCU.

4. Slide the nRF power source switch (SW9) to the VDD position (default)

### Building and Flashing the nPZero application project
Once the hardware is set up, follow these steps to build, flash, and debug the nPZero application project:

1. Set up the **nRF Connect Extension** for **Visual Studio Code** by following these instructions:
    - [nRF Connect for VS Code Tutorials](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code/Tutorials)
    - [nRF Connect for VS Code Documentation](https://docs.nordicsemi.com/bundle/nrf-connect-vscode/page/index.html)
2. Select `Open an existing application` in Visual Studio Code using the nRF Connect Extension to add the nPZero application project
3. Use the nPZero Configurator to configure code examples for peripherals and ADC functionality
4. Insert the code generated by the nPZero Configurator into the `main.c` file in the project directory
5. Build and flash the nPZero application project in Visual Studio Code using the nRF Connect Extension
6. Open a terminal emulator (e.g., PuTTY) and connect to the used COM port with the following UART settings:
    - Baud rate: 115200
    - Data bits: 8
    - Stop bits: 1
    - Parity: None
    - Flow control: None

## Using the nPZero Configurator
This project is compatible with the nPZero Configurator, allowing the configuration to be established directly in the tool:
1) Configure the device as required in the nPZero Configurator
2) Ensure the *Default (nRF52)* template is selected in the *Generated Code* section
3) Click on Save As in the *Generated code* section, and replace the *main.c* file in the *src* folder of this project
   (When making subsequent changes to the configuration, it is sufficient to press *Save* in the *Generated Code* section and the *main.c* file will be updated automatically)
4) Build and flash the code in VSCode as normal

## Generate Doxygen Documentation
Run ``doxygen Doxyfile`` and the documentation will create a html folder which includes a index.html file.
