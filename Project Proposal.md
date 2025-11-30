
Smart Environmental Health Monitoring System 
Objective: 
To design a microcontroller-based system that monitors indoor environmental conditions such as air quality, temperature, humidity, light level, noise level, human presence, and window status. The system will automatically control a ventilation fan and give visual feedback to maintain a healthy and energy-efficient environment. 
System Functions 
Sensors Used 
• DHT11: Temperature & humidity, MQ-135: Air quality (gas detection), LDR: Light intensity, Microphone sensor: Noise level, PIR sensor: Human presence detection, Reed switch: Window/door open-close detection 
Outputs 
• Fan (PWM controlled): Adjusts speed automatically based on air quality and room usage 
• RGB LED: Indicates air quality status (Green–Good, Yellow–Moderate, Red–Poor) 
• LCD (I2C): Shows live sensor data and warnings 
• Bluetooth (HC-05): Sends data wirelessly to phone or PC 
Key Features 
• Calculates Indoor Air Quality (IAQ) Score, detects if the room is occupied (PIR sensor), Detects if window is open (natural ventilation active) 
• Monitors noise level and activates silent mode if necessary (The system continuously monitors the ambient noise level via the microphone sensor (ADC). If the noise level exceeds a predetermined threshold value (e.g. 2500 raw ADC value), the system automatically activates 'Silent Mode'. In this case, regardless of the air quality level, the fan speed is limited to a maximum of 50% duty cycle to prevent additional noise pollution.)
• Automatic fan control based on sensor data, Energy saving mode: If the room is empty, the system reduces power consumption, Bluetooth monitoring: Real-time data transmitted as JSON 
Hardware List 
STM32F407 Microcontroller, DHT11 sensor, MQ-135 sensor, LDR, Microphone module, PIR motion sensor, Reed switch ,HC-05 Bluetooth module, 16x2 I2C LCD, RGB LED, Mini fan + transistor driver, Push button, 5V power supply module 
Expected Outcome 
A working prototype that: 
• Monitors multiple environmental factors 
• Makes automatic decisions to control ventilation 
• Improves air quality and energy efficiency 
• Displays and transmits real-time data 
This project provides a practical and functional smart monitoring system using STM32. It integrates multiple sensors and control mechanisms in a simple but effective way to improve indoor environmental health and comfort. 
