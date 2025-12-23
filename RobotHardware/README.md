# Hardware and Platforms

## Current Hardware Approach


For connection between Unreal Engine (Dekstop or Android phone) and the Robot Hardware, simple TCP is used. 
The ESP32 either starts a server or connects to a network (phone's hotspot), on the Unreal side a very simple ActorComponent and node setup is used to connect as client.
The simple TCP connection allows easy connection to other microcontrollers, f.i. RasperryPi based ones.

### Example Arduino-based implementation:

`OpenAnimal_Mouse.ino` a single-file Arduino script implementation for
* Motor or servo commands
* Reading sensor data and optionally an inertia measurement unit
* Transmitting images
+ additional minor details for being mostly independent of libraries and increasing connection robustness


#### Example Hardware

The preferred hardware is the ESP32, due to its low-cost and built-in Wi-Fi. 

* Freenove 4WD Car Kit (ESP32-CAM with included camera)
* Plus an optional IMU (ICM-20948) for Rotation measurements (UE Blueprint does have access to the phones IMU, but not the magnetometer without external libraries)
* The plugin is designed to be largely camera independent and examples will focus on most common models, like the OV2640 camera.


### USB-Serial (currently not continued)

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
