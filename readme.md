# coregwUART

This is a CubeMX example to use a STM32 H723 as a bidirectional (3.3v) TTL UART <=> USB converter.
It was specifically optimized to transfer UART messages from a WitMotion IMU, but is could be adapted for other uses.

## Demo

![Demo](media/demo.mp4)

## How to use

Install CubeMXIde and import this project.


Plug in a WitMotion IMU to the STM32 board and a USB cable.
You can view data with your favorite imu terminal (I use Termite).


This demo uses the USB ports (PA11 and PA12) and the UART 7 (PE7 and PE8).


## Ressources

[Send and Receive data to PC without UART (STM32 USB COM)](https://controllerstech.com/send-and-receive-data-to-pc-without-uart-stm32-usb-com/)

