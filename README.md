# Intelligent-Safety-System-Assisting-Cyclist

Project repository for HKUST ELEC 3300 Project

##
Features:

###
- Accident Detection and Alert: Detect Fall 
- Rear Vehicle Detection and Alert w/ Tail Light Indication
- RPM Counter, Speed indicator w/ Tail Light Indication
- Climb uphill / Go downhill motion detection w/ advice
- Turn signal w/ Tail Light Indication
- Touch Screen UI

##
Hardware:

###
- STM32F103VET6 MINI-V3 Wild Fire development board
- MPU6050 6-axis (Gyro + Accelerometer) motion sensor
- VL53L1X long distance ranging Time-of-Flight sensor
- A3144 Hall Effect Sensor
- 2.8 inches TFT Touch Screen (Resolution: 320*240) with ILI9341_LCD and XPT2046 chip

##
Software and development environment:

###
- STM32CubeIDE (v1.9.0)
- Debug with ST-Linker

##
Technical details

###
Most of the Serial Communication between MCU and sensors / other components are via I2C and GPIO (w/ some UART & DMA)
To be Updated...
