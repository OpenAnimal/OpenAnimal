# Hardware and Platforms

## Current Hardware Approach


For between Unreal Engine (Android phone) and the robot we use HTTP queries, which is the intended and recommended way to communicate. 
The preferred hardware is the ESP32, due to its low-cost and built-in Wi-Fi. 

Example Arduino-based implementations are or will be provided soon, showing how to transmit:
* Motor or servo commands
* Reading IMU or sensor data
* Transmitting images


In the future we will release more advanced example codes. And we might expand to additional platforms like various Arduino boards (in particular M5 ESP32) and potentially Raspberry Pi, or different communication methods. The overall aim is to support inexpensive and widely accessible hardware.



#### Example Hardware
* Freenove 4WD Car Kit for ESP32 CAM 
* Plus an IMU for Rotation measurements (in particular with magnetometer, like ICM-20948)
(By default UE Blueprint does have access to the phones IMU but not the magnetometer.)
* The plugin is designed to be largely camera independent and examples will focus on most common models, like the OV2640 camera.


### USB-Serial

The option to connect Unreal to Arduino via USB currently on hold, as the [SerialCOM](https://github.com/videofeedback/Unreal_Engine_SerialCOM_Plugin) plugin doesn't compile for android yet.
Also the USB Ports are freed for battery charging. 
The SerialCOM Plugin for USB connection still works for UE5.5 on Windows and a basic script for Arduino is attached for completeness.



### Typical Complete Workflow

Place your Android phone on the robot.  
Connect the phone to the ESP32 over Wi-Fi.  
Launch OpenAnimal (OpenBrain) on the phone.  
The phone uses its neural networks and microphone to perceive the environment.  
The phone sends motor/servo commands to the robot.  
The robot sends sensor data back.  
Higher-cognition modules run in the background when you request advanced analysis.
