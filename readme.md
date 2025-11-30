# STM32 Smart Environmental Health Monitoring System Project

Project Partners: 

Uzeyir YAMAN (2111011074) 

Ensar Taha AYAS (2111011094) 

Muhammed Yiğithan YILDIZ (2211011046) 

1. Project Objective
The project aims to develop a microcontroller-operated set-up that will monitor the indoor environmental conditions such as air quality, temperature, humidity, light and noise level, and person count with the use of STM32F407. The set-up controls the above mentioned parameters by turning the fan (fan speed) on or off according to the sensors, hence maintaining a healthy and energy-efficient environment, while providing real time visual feedback via LCD/LEDs and remote monitoring through Bluetooth.

2. Key Features and Functionality
IAQ (Indoor Air Quality) Calculation: The evaluation of air quality is done by assigning a quality score (0-500).

Smart Ventilation: Fan speed is controlled with PWM and is directly proportional to levels of air quality.

Energy Efficiency:

Occupancy Detection: A PIR sensor is used to detect the presence of a person in the room and hence reduce power if the room is vacant.

Window Detection: A Reed switch is employed to instantly turn off the fan when a window is opened (natural ventilation takes priority).

Quiet Mode: The noise level is monitored through a microphone; fan speed is restricted if the surrounding is noisy.

Connectivity: Sends all sensor data to a mobile device in JSON format via Bluetooth (UART) with transmission support.

User Interface: Shows real-time data on an I2C LCD and communicates status through RGB LEDs.

3. Hardware and Interfaces
Microcontroller: STM32F407G-DISC1

Sensors: DHT11 (Temperature/Humidity), MQ-135 (Gas), LDR (Light), Microphone (Noise), PIR (Motion), Reed Switch (Window).

Actuators: DC Fan (PWM controlled), RGB LEDs embedded.

Communication: HC-05 Bluetooth Module (UART), LCD 16x2 (I2C).

4. Software Implementation Strategy
Hybrid Architecture: The project employs a combination of HAL library for Recording Level (CMSIS) programming for complex communication protocols (I2C, UART, ADC) and high-speed Digital GPIO operations (PIR, Reed Switch, LEDs) to ensure performance and show low-level hardware control.
