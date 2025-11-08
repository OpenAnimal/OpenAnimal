

## Hardware and Platforms

Note - the current SerialCOM.uplugin file still says version 5.4 for plugin version 5.5. Change it manually to skip the warning at startup.

For connecting UE Brain to a platform, we provide the Serial Interface for communicating with Arduino. 

The SerialCom Example file shows the bytes that send control commands, image data and sensor signals.

This is kept very simple for now, but we will release more advanced example codes. This might include different platforms, mainly Arduino (our development was based on M5 ESP32), Rasperry Pi probably at some point via different communication methods. In general we are aiming for very low cost and widely available hardware.

The SerialCom Plugin is not yet compatible for Android, but will be made compatible at some point.



Example for ESP32 

Freenove 4WD Car Kit for ESP32 CAM       (69$)
ESP32 runs arduino code and provides WiFi
 
Plus an IMU for Rotation measurements icm-20948 (14$)