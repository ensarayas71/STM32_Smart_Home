# STM32 Smart Environmental Health Monitoring System Project

Project Partners: 

Uzeyir YAMAN (2111011074) 

Ensar Taha AYAS (2111011094) 

Muhammed Yiğithan YILDIZ (2211011046) 

1. Abstract
 The Smart Environmental Health Monitoring System (SEHMS) is an innovative embedded system created to optimize both indoor air quality and energy efficiency through constant real-time monitoring of temperature, humidity, gas, light intensity, noise level, human occupancy, and window status using the STM32F407 microcontroller.
 Unlike traditional monitoring systems that only read and record I/O activity, SEHMS utilizes a fully autonomous decision-making algorithm that will maximize natural ventilation when a window is open; adjust operation mode based on how many people are in a room; and use Pulse Width Modulation (PWM) to drive the fan for a ventilation system using Hybrid Software Architecture. This hybrid software architecture incorporates the speed and efficiency of low-level Register-Level (CMSIS) programming for all critical I/O functions, while also providing access to the recommended and more complex peripheral functions using the easy-to-implement HAL Libraries.
2. Key Features & Functionalities
   a. Indoor Air Quality (IAQ) Analysis
   Sensor: MQ-135 Gas Sensor.
   Logic: The analog values (0-4095) from ADC are received by the system. A tailor-made algorithm translates this unrefined information into a consistent Indoor Air Quality (IAQ) Score ranging between 0 and 500. 
   Action: If the score exceeds critical thresholds (>300), the ventilation fan accelerates to maximum speed to evacuate pollutants.

   b. Intelligent Energy Management
   Window Detection (Priority Override): A Reed Switch is employed to determine whether a window is in the open position; when a window is in the open position, the Reed Switch will instantly turn off the fan to save energy (Natural Ventilation Mode).
   Occupancy Detection: A PIR Sensor detects a person in the room. If there is no person in the room and the air is not too bad, the unit will go into Sleep Mode to save energy.

   c. Noise-Aware Silent Mode
   Sensor: Analog Microphone Module (KY-037).
   Logic: Ambient noise is sampled periodically by the system; if the ambient noise level exceeds a predetermined limit (as determined by the environmental recording), Quiet Mode will be active and the speed of the fans will remain at 50% even when the air quality may be changing slightly.

   d. Precision Climate Monitoring
   Sensor: DHT11 Temperature & Humidity Sensor.
   Implementation: A custom "bit-banging" driver was written using a DWT (Data Track Point And Track) loop counter for microsecond precision timing, bypassing standard HAL delays for higher accuracy.

   e. IoT Connectivity & Remote Monitoring
   Protocol: UART (Universal Asynchronous Receiver-Transmitter).
   Format: JSON (JavaScript Object Notation).
   Function: Real time data is transmitted to a paired Bluetooth device (PC/Phone) twice per second.

3. Hardware Architecture & Pin Configuration
   The system is based on STM32F407VGT6 MCU. Detailed pin mapping is below:
   <img width="632" height="290" alt="image" src="https://github.com/user-attachments/assets/489d936f-d8ef-4796-b387-b50bcf2a2ff7" />

4. Software Implementation Strategy
   The project demonstrates a professional hybrid coding approach that meets advanced engineering requirements.
   a. Register Level Programming (CMSIS)
   For time-critical and frequent operations, we bypassed the HAL overhead and accessed memory addresses directly.
   Digital Inputs (PIR, Window): Accessed via GPIOx->IDR.
   Digital Outputs (LEDs): Controlled via GPIOx->ODR for instant status updates.
   This reduces CPU cycles and proves understanding of the ARM Cortex-M4 architecture.

   b. HAL Library Usage
   Optimized and used HAL drivers for complex communication protocols.
   ADC (Analog-to-Digital): Configured with 3-Cycle sampling time for fast data acquisition.
   PWM (Pulse Width Modulation): TIM3 is configured with a period of 1000, allowing for 0.1% precise speed control steps.
   I2C and UART: Used for stable LCD and Bluetooth communication.

   c. System Flowchart
   Initialization: Clock (HSE), Peripherals, and LCD setup.
   Sensors Read: DHT11 (One-Wire), ADC Group (DMA/Polled), GPIO Registers.
   Data Processing: Calculate IAQ score, check thresholds.
   Decision Tree: If Window Open -> STOP FAN (Blue LED) / Else If High Pollution -> MAX SPEED (Red LED) / Else If Noise Detected -> LIMIT SPEED (Silent Mode)
   Output: Update LCD (Page 1/2), Send JSON via Bluetooth.
   Loop: Repeat every 1000ms.

5. How to Build and Run
   a. Hardware Setup: Based on the pin configuration table provided above, connect the sensors. The LCD must be supplied with a power source (5V) instead of 3.3V for optimal visibility. Connect the HC-05 RX to STM32 TX (PC10) and vice versa.
   b. Software Setup: Download this repository. Open the project in STM32CubeIDE, then click the Hammer icon to build it. Connect your STM32F4 Discovery Board with USB. Finally, click Run/Debug to load the firmware onto your board.
   c. Monitoring: Make sure the LCD screen will change every two seconds between "Temp/Sound" and "IAQ/Light" on your device. Use a Bluetooth Terminal application on your smartphone and connect to HC-05 to see live JSON data stream.

6. Future Improvements
   Add an RTOS (real-time operating system) for task scheduling, and connect an ESP8266 so you can push data up to a web-based dashboard (IoT Cloud) and include PID (proportional, integral, derivative) controlling to make smooth fan-speed changes.





 
