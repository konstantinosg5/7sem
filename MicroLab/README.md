# 📌 Microprocessors Course - Exercises Documentation

## 📖 Overview
This document provides a structured overview of the **Microprocessors Course**, covering **eight sets of exercises** focusing on embedded systems programming with the **ATmega328PB** microcontroller. The exercises include implementations in **C and Assembly**, utilizing various peripherals and communication protocols.

## 📂 Repository Contents

### 🔧 Exercise 3: Timers and ADC Converters
- **Concepts Covered:**
  - ATmega328PB **timers** (Timer0, Timer1, Timer2)
  - **PWM (Pulse Width Modulation)** ⚡
  - **ADC (Analog-to-Digital Converter)** configuration and usage 🎛️
  - Interrupt handling for timers and ADC
- **Practical Implementation:**
  - Generating PWM signals for LED dimming 💡
  - Reading analog values from sensors 📊

### 🔍 Exercise 4: 16x2 LCD Display & Fire Detection System
- **Concepts Covered:**
  - **16x2 LCD Display** 📟
  - Fire detection sensor integration 🔥
  - Data visualization on LCD
- **Practical Implementation:**
  - Displaying messages and sensor readings
  - Simulating an alert system for fire detection 🚨

### 🔗 Exercise 5: TWI (I2C) Protocol
- **Concepts Covered:**
  - **TWI (Two-Wire Interface) Protocol** 🔄
  - Master-slave communication
- **Practical Implementation:**
  - Configuring ATmega328PB as an I2C master 🖧
  - Communicating with external I2C peripherals like the LCD

### 🎹 Exercise 6: PCA9555D I/O Expander and Keyboards
- **Concepts Covered:**
  - **PCA9555D I/O Expander**
  - Interfacing external keyboards ⌨️
- **Practical Implementation:**
  - Using the PCA9555D for GPIO expansion
  - Implementing a keypad interface

### 🌡️ Exercise 7: DS1820 Temperature Sensor & One-Wire Protocol
- **Concepts Covered:**
  - **One-Wire Communication Protocol** 🔗
  - **DS1820 Digital Temperature Sensor** 🌡️
- **Practical Implementation:**
  - Reading temperature values via One-Wire protocol 📡
  - Displaying results on an LCD 📟

### 🏥 Exercise 8: Hospital IoT Patient Monitoring System
- **Final Project integrating all previous exercises** 🏗️
- **Concepts Covered:**
  - ADC, Timers, LCD Display, I2C, One-Wire, and UART
  - **ESP8266 Wi-Fi Module** (UART communication) 📡
  - **Real-time patient monitoring** 🏥
- **Practical Implementation:**
  - Collecting and transmitting **blood pressure, temperature, and patient status** data 🏥
  - Sending data to a remote server for **analysis and alerts** 📡
  - Displaying patient status on an LCD 📟

## 🛠️ Software & Tools
- **AVR Toolchain** (GCC for AVR) 💾
- **Arduino IDE / Atmel Studio** 🖥️
- **ESP8266 AT firmware** 📶


## ✅ Conclusion
This document summarizes the core concepts and implementations from the microprocessors course. The final project consolidates all topics into a **fully functional IoT-based patient monitoring system**. 🏥📡
