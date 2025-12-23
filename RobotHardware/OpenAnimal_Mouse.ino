/**********************************************************************
  OpenAnimal ESP32 Camera + Motor + Control
  Target     : 4WD car with PCA9685 + ESP32-CAM Wrover module
  WiFi       : AP "Sunshine" / "Sunshine"
  Ports      : 4000 = commands + sensors (text)
               7000 = JPEG stream (4-byte length + JPEG)

  Most functions are available in the the official esp32 espressif library. 
  Single file implementation based partly on Freenove 4WD. Tutorial and Libraries: 
  github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32 (Mainly PCA9685)
  
  
**********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "esp_adc_cal.h"
#include <PCA9685.h>   // 16-Channel Servo driver PCA9685 Library

///////////////////////////////////
/////////// Setup Area ////////////
///////////////////////////////////




// ===================================================================
// Motor Setup
// ===================================================================
constexpr int NUM_SERVOS = 4;
// Max PWM magnitude used with Motor_Move
const int16_t MOTOR_MAX_PWM = 2000;  // Might be different for servos

// If the direction is reversed, change 1 to -1 (per wheel)
#define MOTOR_1_DIRECTION     1
#define MOTOR_2_DIRECTION     1
#define MOTOR_3_DIRECTION     1
#define MOTOR_4_DIRECTION     1

// Global inversion so that +1.0 from UE means "forward" on the floor
bool gInvertAllMotors = false;



// Debug Print Flag // 
// Set to 'true' to enable serial monitor printing of motor and sensor signals. 
// Set to false for max performance
bool gDebugSerialPrintIMU = false;


// ===================================================================
// WiFi Config
// ===================================================================

// MODE_AP (ESP32 Wifi) or MODE_STA (External Wifi)
#define WIFI_MODE MODE_STA    

// AP mode
// ESP default IP: 192.168.4.1
const char* ssid_AP      = "Sunshine";
const char* password_AP  = "Sunshine";

// STA (external Router) mode (switch WIFI_MODE to MODE_STA)
// To connect to a router or f.i. a hotspot created by the phone. 
// IP might be change. To prevent this, one could either set a static IP, 
// or a small discovery process (mDNS or UDP broadcast), but this is skipped for now.
// Note, Hotspot needs to be 2.4 GHz+WPA2 (not 5GHz+WPA3)
const char* ssid_Router     = "Galaxy S10e9786";
const char* password_Router = "mhmwmhm1";

// Servers
WiFiServer server_Cmd(4000);     // Commands + sensors (text)
WiFiServer server_Camera(7000);  // JPEG stream (binary)



// ===================================================================
// Camera State
// ===================================================================
framesize_t frame_size = FRAMESIZE_VGA; // 640x480 (is what UE's receiving material currently expects)
bool        videoSendingFlag  = false;  // passive / internal
int         videoFps   = 10;            // default FPS
unsigned long lastFrameMs = 0;



// ===================================================================
// Sensor State (stub)
// ===================================================================
unsigned long lastSensorMs   = 0;
unsigned long sensorPeriodMs = 100;  // 10 Hz sensor updates



// ===================================================================
// Battery (optional)
// ===================================================================
#define ENABLE_BATTERY_MONITOR 1
#define PIN_BATTERY        32        //Set the battery detection voltage pin
#define LOW_VOLTAGE_VALUE  2100      //Set the minimum battery voltage
#define DEFAULT_VREF    1100         // mV, ADC reference for calibration

/////////////////////////////////
//////// Setup Area End //////////
/////////////////////////////////


void debugScanNetworks()
{
  Serial.println("Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("No networks found.");
    return;
  }

  Serial.printf("Found %d networks:\n", n);
  for (int i = 0; i < n; ++i) {
    Serial.printf(
      "  %2d: SSID='%s', RSSI=%d dBm, channel=%d\n",
      i + 1,
      WiFi.SSID(i).c_str(),
      WiFi.RSSI(i),
      WiFi.channel(i)
    );
  }
}

// ===================================================================
// Function Declarations
// ===================================================================
bool cameraSetup(void);
void WiFi_Setup_AP();
void WiFi_Setup_STA();
void loopTask_Camera(void* pvParameters);
void handleCommandLine(const String& line, WiFiClient& client);
void applyServoTargets();
void sendSensorData(WiFiClient& client);

#if ENABLE_BATTERY_MONITOR
  int   Get_Battery_Voltage_ADC(void);
  float Get_Battery_Voltage(void);
  void  Set_Battery_Coefficient(float coefficient);
  void  Setup_Battery_Monitor(void);
#endif

#if ENABLE_IMU
  void setupIMU();
#endif



// ===================================================================
// Motor / Servo driver PCA9685 
// ===================================================================
// Incoming:   CMD_SERVO#4#v0#v1#v2#v3
// where v0 to v3 are normalized floats in [-1.0, +1.0]
// Order (presumably): [front_left, back_left, front_right, back_right]

float ServoTargets[NUM_SERVOS];
int   NumServos = NUM_SERVOS;

// I2C pins to the PCA9685 motor board
#define PCA9685_MOTOR_SDA 13
#define PCA9685_MOTOR_SCL 14

#ifndef PCA9685_ADDRESS
#define PCA9685_ADDRESS   0x5F
#endif

#define MOTOR_FREQUENCY   1000       // Hz
#define MOTOR_SPEED_MIN   -4095
#define MOTOR_SPEED_MAX   4095

// PCA9685 output channels for each motor H-bridge input
#define PIN_MOTOR_M1_IN1  15
#define PIN_MOTOR_M1_IN2  14
#define PIN_MOTOR_M2_IN1  9
#define PIN_MOTOR_M2_IN2  8
#define PIN_MOTOR_M3_IN1  12
#define PIN_MOTOR_M3_IN2  13
#define PIN_MOTOR_M4_IN1  10
#define PIN_MOTOR_M4_IN2  11

// Global PCA9685 object used for motors
PCA9685 PCA9685_MOTOR;

// Close the PCA9685 public address (same as in Freenove code)
void PCA9685_Close_Com_Address(void)
{
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

// Low-level init of PCA9685 chip for motors
void Motor_Setup(void)
{
  Wire.begin(PCA9685_MOTOR_SDA, PCA9685_MOTOR_SCL);
  PCA9685_MOTOR.setupSingleDevice(Wire, PCA9685_ADDRESS);
  PCA9685_Close_Com_Address();
  PCA9685_MOTOR.setToFrequency(MOTOR_FREQUENCY);
}

// Simple wrapper so the rest of your code can keep calling PCA9685_Setup()
void PCA9685_Setup(void)
{
  Motor_Setup();
}

// Actual Motor_Move implementation (unchanged logic from Freenove)
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed)
{
  if (PCA9685_MOTOR.getFrequency() != MOTOR_FREQUENCY)
    Motor_Setup();

  m1_speed = MOTOR_1_DIRECTION * constrain(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m2_speed = MOTOR_2_DIRECTION * constrain(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m3_speed = MOTOR_3_DIRECTION * constrain(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m4_speed = MOTOR_4_DIRECTION * constrain(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  // M1
  if (m1_speed >= 0)
  {
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M1_IN1, m1_speed);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
  }
  else
  {
    m1_speed = -m1_speed;
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M1_IN2, m1_speed);
  }

  // M2
  if (m2_speed >= 0)
  {
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M2_IN1, m2_speed);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
  }
  else
  {
    m2_speed = -m2_speed;
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M2_IN2, m2_speed);
  }

  // M3
  if (m3_speed >= 0)
  {
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M3_IN1, m3_speed);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
  }
  else
  {
    m3_speed = -m3_speed;
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M3_IN2, m3_speed);
  }

  // M4
  if (m4_speed >= 0)
  {
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M4_IN1, m4_speed);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
  }
  else
  {
    m4_speed = -m4_speed;
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
    PCA9685_MOTOR.setChannelPulseWidth(PIN_MOTOR_M4_IN2, m4_speed);
  }
}





// ===================================================================
// Camera Pins
// ===================================================================
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22




// ===================================================================
// Camera init
// ===================================================================
bool cameraSetup(void)
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_1;
  config.ledc_timer   = LEDC_TIMER_1;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = frame_size;
  config.jpeg_quality = 10;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  Serial.println("Camera initialization complete!");
  return true;
}


// ===================================================================
// Wifi Mode & protocol
// ===================================================================
#define MODE_AP  1
#define MODE_STA 2

#define CMD_VIDEO           "CMD_VIDEO"
#define CMD_SERVO           "CMD_SERVO"   // used for 4WD motor control (normalized floats)
#define CMD_FPS             "CMD_FPS"
#define INTERVAL_CHAR       '#'
#define ENTER               '\n'


void WiFi_Setup_AP()
{
  Serial.println("\nConfiguring WiFi in AP Mode...");
  WiFi.disconnect(true); delay(300);  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_AP, password_AP);

  Serial.println("--- WiFi AP Started ---");
  Serial.print("SSID: ");     Serial.println(ssid_AP);
  Serial.print("Password: "); Serial.println(password_AP);
  Serial.print("IP: ");       Serial.println(WiFi.softAPIP());
}

void WiFi_Setup_STA()
{
  Serial.println("\nConfiguring WiFi in STA Mode...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  while (true)
  {
    WiFi.disconnect(true);

    Serial.printf("Connecting to router: %s\n", ssid_Router);
    WiFi.begin(ssid_Router, password_Router);

    unsigned long startAttemptTime = millis();
    const unsigned long connectTimeoutMs = 15000; // 15 seconds

    while (WiFi.status() != WL_CONNECTED &&
           millis() - startAttemptTime < connectTimeoutMs)
    {
      Serial.print(".");
      delay(500);
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("--- WiFi STA Connected ---");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      break;
    }

    Serial.println("WiFi connect failed, retrying in 5 seconds...");
    delay(5000);
  }
}


// ===================================================================
// Optional IMU (ICM-20948 via SparkFun Library)
// ===================================================================
// Note: Adafruit libraries conflict with esp_camera.h 
// due to "sensor_t" redefinition. SparkFun does not have this conflict.
#define ENABLE_IMU 1

#if ENABLE_IMU
  #include <ICM_20948.h> // SparkFun ICM 20948 in Library Manager
  
  ICM_20948_I2C icm; 
  bool gImuPresent = false;

  // --- Gyro bias globals ---
  bool  gGyroCalibrated = false;
  float gGyroBiasX = 0.0f;
  float gGyroBiasY = 0.0f;
  float gGyroBiasZ = 0.0f;

  // IMU stall detection (last good read)
  unsigned long gImuLastGoodReadMs = 0;
  const unsigned long IMU_STALL_TIMEOUT_MS = 500; // ms

  void setupIMU()
  {
    Serial.println("Initializing ICM-20948 IMU ...");

    // I2C is already started on pins 13/14 inside PCA9685_Setup() via Wire.begin().    
    // We just probe the IMU on that same bus.
    Serial.print("Probing 0x69... "); // default address 0x69 or 0x68 depending on AD0 pin
    icm.begin(Wire, 1);    
      
    if (icm.status != ICM_20948_Stat_Ok) {
      Serial.print("Failed. Probing 0x68... ");
      icm.begin(Wire, 0); 
    }

    if (icm.status == ICM_20948_Stat_Ok) {
      gImuPresent = true;
      Serial.println("DETECTED!");

      Serial.println("ICM-20948 initialization complete!");

      // Optional tuning – defaults should be OK:
      // icm.setAccelRange(g_4);     // Sets Accelerometer to +/- 4g
      // icm.setGyroRange(dps_500);  // Sets Gyroscope to +/- 500 DPS
    }
    else {
      gImuPresent = false;
      Serial.printf("Failed. Status: %s. IMU disabled.\n", icm.statusString());
    }
  }

  // --- Gyro bias calibration: keep the robot perfectly still! ---
  void calibrateGyro()
  {
    if (!gImuPresent) return;

    const int   N             = 300;   // fewer samples = faster
    const int   SAMPLE_DELAY  = 5;     // ms between samples

    Serial.println("\n==== Gyro bias calibration ====");
    Serial.println("Keep the robot/IMU completely still...");
    delay(1000);

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < N; ++i)
    {
      icm.getAGMT();   // will update gyrX/Y/Z
      sumX += icm.gyrX();
      sumY += icm.gyrY();
      sumZ += icm.gyrZ();
      delay(SAMPLE_DELAY);
    }

    gGyroBiasX = sumX / N;
    gGyroBiasY = sumY / N;
    gGyroBiasZ = sumZ / N;
    gGyroCalibrated = true;

    Serial.printf("Gyro bias (dps): X=%.3f  Y=%.3f  Z=%.3f\n\n",
                  gGyroBiasX, gGyroBiasY, gGyroBiasZ);
  }

#endif // ENABLE_IMU




// ===================================================================
// Motor control via Motor_Move (PCA9685)
// ===================================================================
void applyServoTargets()
{
  // We expect at least 4 values for the 4WD car
  if (NumServos < 4)
  {
    Serial.println("applyServoTargets: Not enough values, expected at least 4.");
    return;
  }

  int16_t motorVals[4];

  for (int i = 0; i < 4; ++i)
  {
    float v = ServoTargets[i];

    // Clamp to [-1,1]
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;

    float scaled = v * (float)MOTOR_MAX_PWM;

    if (gInvertAllMotors)
    {
      scaled = -scaled;
    }

    motorVals[i] = (int16_t)scaled;
  }

  Motor_Move(motorVals[0], motorVals[1], motorVals[2], motorVals[3]);

  if (gDebugSerialPrintIMU) {
    Serial.print("Motor_Move: ");
    Serial.print(motorVals[0]); Serial.print(", ");
    Serial.print(motorVals[1]); Serial.print(", ");
    Serial.print(motorVals[2]); Serial.print(", ");
    Serial.println(motorVals[3]);
  }
}

// ===================================================================
// Sensor sending (stub)
// ===================================================================
void sendSensorData(WiFiClient& client)
{
  // Still placeholders for now
  float distFront = -1.0f;
  float distBack  = -1.0f;

  #if ENABLE_BATTERY_MONITOR
    float battery = Get_Battery_Voltage();
  #else
    float battery = -1.0f;   // "unknown"
  #endif

  // Start SENSOR line
  client.print("SENSOR#dist_front=");
  client.print(distFront, 2);
  client.print("#dist_back=");
  client.print(distBack, 2);
  client.print("#battery=");
  client.print(battery, 2);

  // Optional IMU data – appended as extra key/value pairs
  #if ENABLE_IMU
    // Thermal Correction Constant: For heat generated by the IMU's self-heating.
    const float IMU_TEMP_CORRECTION_C = 7.0f;

    if (gImuPresent)
    {
      unsigned long now = millis();
      bool imuOkThisCycle = false;

      // Always try one update; rely on status instead of dataReady()
      icm.getAGMT();

      if (icm.status == ICM_20948_Stat_Ok)
      {
        gImuLastGoodReadMs = now;
        imuOkThisCycle = (gImuLastGoodReadMs != 0);
      }
      else
      {
        // If IMU has been bad for a while, try to re-init + recalibrate
        if (gImuLastGoodReadMs != 0 &&
            (now - gImuLastGoodReadMs) > IMU_STALL_TIMEOUT_MS)
        {
          Serial.println("IMU stalled, re-initializing ICM-20948...");
          setupIMU();
          if (gImuPresent)
          {
            calibrateGyro();
            gImuLastGoodReadMs = millis();
            imuOkThisCycle = true;
            Serial.println("IMU re-init OK.");
          }
          else
          {
            Serial.println("IMU re-init failed, IMU disabled.");
          }
        }
      }

      // If we still don't have a good read, just skip sending IMU fields
      if (imuOkThisCycle)
      {
        // Temperature (°C)
        float raw_temp = icm.temp();
        float corrected_temp = raw_temp - IMU_TEMP_CORRECTION_C;
        client.print("#imu_temp=");
        client.print(corrected_temp, 2);

        // ACCELERATION: mg -> g -> m/s^2
        float ax_g = icm.accX() / 1000.0f;
        float ay_g = icm.accY() / 1000.0f;
        float az_g = icm.accZ() / 1000.0f;

        const float G_ACCEL = 9.80665f;
        float ax_ms2 = ax_g * G_ACCEL;
        float ay_ms2 = ay_g * G_ACCEL;
        float az_ms2 = az_g * G_ACCEL;

        client.print("#imu_ax="); client.print(ax_ms2, 3); 
        client.print("#imu_ay="); client.print(ay_ms2, 3);
        client.print("#imu_az="); client.print(az_ms2, 3);

        // Gyro (DPS - Degrees Per Second), apply bias
        float rawGx = icm.gyrX();
        float rawGy = icm.gyrY();
        float rawGz = icm.gyrZ();

        float calGx = rawGx;
        float calGy = rawGy;
        float calGz = rawGz;
        if (gGyroCalibrated)
        {
          calGx -= gGyroBiasX;
          calGy -= gGyroBiasY;
          calGz -= gGyroBiasZ;
        }

        client.print("#imu_gx="); client.print(calGx, 3);
        client.print("#imu_gy="); client.print(calGy, 3);
        client.print("#imu_gz="); client.print(calGz, 3);

        // Magnetometer (uT)
        client.print("#imu_mx="); client.print(icm.magX(), 3);
        client.print("#imu_my="); client.print(icm.magY(), 3);
        client.print("#imu_mz="); client.print(icm.magZ(), 3);

        if (gDebugSerialPrintIMU) {
          // Print nicely every 500 ms when debug is on
          static unsigned long lastPrint = 0;
          if (now - lastPrint >= 500)
          {
              lastPrint = now;

              Serial.println("------------ ICM-20948 data ------------");

              Serial.print("Temp (uncorrected): ");
              Serial.print(icm.temp());
              Serial.println(" °C");

              Serial.print("Accel X: "); Serial.print(icm.accX(), 2); Serial.print(" mg\t");
              Serial.print("Y: ");
              Serial.print(icm.accY(), 2); Serial.print(" mg\t");
              Serial.print("Z: ");
              Serial.print(icm.accZ(), 2); Serial.println(" mg");

              Serial.print("Gyro X: "); Serial.print(icm.gyrX(), 2); Serial.print(" dps\t");
              Serial.print("Y: ");
              Serial.print(icm.gyrY(), 2); Serial.print(" dps\t");
              Serial.print("Z: ");
              Serial.print(icm.gyrZ(), 2); Serial.println(" dps");

              Serial.print("Mag X: "); Serial.print(icm.magX(), 2); Serial.print(" uT\t");
              Serial.print("Y: ");
              Serial.print(icm.magY(), 2); Serial.print(" uT\t");
              Serial.print("Z: ");
              Serial.print(icm.magZ(), 2); Serial.println(" uT");

              Serial.println("-----------------------------------------");
          }
        }
      } // end if (imuOkThisCycle)
    }
  #endif

  // End of line (always exactly one newline)
  client.print("\n");
}


  

// ===================================================================
// Command parsing
// ===================================================================
void handleCommandLine(const String& line, WiFiClient& client)
{

  if (line.startsWith(CMD_VIDEO))
  {
    int idx = line.indexOf(INTERVAL_CHAR);
    if (idx > 0)
    {
      int v = line.substring(idx + 1).toInt();
      videoSendingFlag = (v == 1);
      Serial.printf("videoSendingFlag = %d\n", videoSendingFlag ? 1 : 0);
    }
  }
  else if (line.startsWith(CMD_FPS))
  {
    int idx = line.indexOf(INTERVAL_CHAR);
    if (idx > 0)
    {
      int fps = line.substring(idx + 1).toInt();
      if (fps > 0 && fps <= 60)
      {
        videoFps = fps;
        Serial.printf("Set video FPS to %d\n", videoFps);
      }
    }
  }
  else if (line.startsWith(CMD_SERVO))
  {
    // Format: CMD_SERVO#N#v0#v1#...#vN-1
    int firstHash  = line.indexOf(INTERVAL_CHAR);
    int secondHash = line.indexOf(INTERVAL_CHAR, firstHash + 1);
    if (firstHash > 0 && secondHash > firstHash)
    {
      int n = line.substring(firstHash + 1, secondHash).toInt();
      n = constrain(n, 0, NumServos);
      NumServos = n;

      int currentPos = secondHash + 1;
      for (int i = 0; i < NumServos; ++i)
      {
        int nextHash = line.indexOf(INTERVAL_CHAR, currentPos);
        String valueStr;
        if (nextHash == -1)
        {
          valueStr = line.substring(currentPos);
        }
        else
        {
          valueStr = line.substring(currentPos, nextHash);
        }
        ServoTargets[i] = valueStr.toFloat();
        if (nextHash == -1) break;
        currentPos = nextHash + 1;
      }

      applyServoTargets();

      // Debug print only if enabled AND at least one motor command != 0
      if (gDebugSerialPrintIMU)
      {
        bool anyNonZero = false;
        for (int i = 0; i < NumServos; ++i)
        {
          if (ServoTargets[i] != 0.0f)   // or use a small epsilon
          {
            anyNonZero = true;
            break;
          }
        }

        if (anyNonZero)
        {
          Serial.print("CMD: ");
          Serial.println(line);
        }
      }
    }
  }
  else
  {
    Serial.println("Unknown command.");
  }
}

// ===================================================================
// Camera task: binary JPEG stream on port 7000
// ===================================================================
void loopTask_Camera(void* pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    WiFiClient client = server_Camera.available();
    if (client)
    {
      Serial.printf("Camera client connected");
      lastFrameMs = millis();

      while (client.connected())
      {
        if (videoSendingFlag)
        {
          unsigned long now = millis();
          unsigned long interval = (videoFps > 0) ? (1000UL / (unsigned long)videoFps) : 0;

          if (interval == 0 || (now - lastFrameMs) >= interval)
          {
            lastFrameMs = now;

            camera_fb_t* fb = esp_camera_fb_get();
            if (fb)
            {
              uint32_t len = fb->len;
              uint8_t header[4];
              header[0] = (len >> 0) & 0xFF;
              header[1] = (len >> 8) & 0xFF;
              header[2] = (len >> 16) & 0xFF;
              header[3] = (len >> 24) & 0xFF;

              client.write(header, 4);
              client.write(fb->buf, fb->len);
              esp_camera_fb_return(fb);
            }
            else
            {
              Serial.println("Failed to get frame");
            }
          }
          else
          {
            delay(1);
          }
        }
        else
        {
          delay(50);
        }
      }

      client.stop();
      Serial.println("Camera client disconnected.");
      videoSendingFlag = false;
    }

    delay(1); // yield a bit
  }
}

// ===================================================================
// Battery Status. Only done if Battery Monitor is set to true.
// ===================================================================
#if ENABLE_BATTERY_MONITOR

  float batteryVoltage = 0;       //Battery voltage variable
  float batteryCoefficient = 4;   //Set the proportional coefficient
  static esp_adc_cal_characteristics_t *adc_chars;
  static const adc_atten_t atten = ADC_ATTEN_DB_12;
  static const adc_unit_t unit = ADC_UNIT_1;

  // Gets the battery ADC value
  int Get_Battery_Voltage_ADC(void)
  {
    long batteryADC = 0; 
    for (int i = 0; i < 5; i++) {
      batteryADC += analogRead(PIN_BATTERY);
    }
    return batteryADC / 5;
  }

  // Get the battery voltage value
  float Get_Battery_Voltage(void)
  {
    int batteryADC = Get_Battery_Voltage_ADC();
    uint32_t voltage_at_pin_mv = esp_adc_cal_raw_to_voltage(batteryADC, adc_chars);
    float batteryVoltage = (voltage_at_pin_mv / 1000.0) * batteryCoefficient;
    return batteryVoltage;
  }

  void Set_Battery_Coefficient(float coefficient)
  {
    batteryCoefficient = coefficient;
  }

  // Battery voltage detection initialization
  void Setup_Battery_Monitor() {
    // Set ADC resolution to 12-bit
    analogSetWidth(12);
    analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  }
#endif

// ===================================================================
// Arduino setup / loop
// ===================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting OpenAnimal ESP32 4WD...");

  // Motor controller (PCA9685) init
  PCA9685_Setup();
  Serial.println("PCA9685 motor controller initialized.");

  #if ENABLE_BATTERY_MONITOR
    // Initialize ADC/etc. for battery reading
    Setup_Battery_Monitor();
    Serial.println("Battery monitor initialized.");
  #endif

  // Optional IMU init (shares same I2C bus as PCA9685)
  #if ENABLE_IMU
    setupIMU();
    calibrateGyro(); 
  #endif

  // WiFi
  if (WIFI_MODE == MODE_AP) {
    Serial.println("Selected WiFi mode: AP");
    WiFi_Setup_AP();
  }
  else if (WIFI_MODE == MODE_STA) {
    Serial.println("Selected WiFi mode: STA");
    WiFi_Setup_STA();
  }
  else {
    Serial.println("Invalid WIFI_MODE. Halting.");
    while (true) { delay(1000); }
  }

  Serial.println("WiFi setup function completed.");
    server_Cmd.begin();


  // Camera init
  if (!cameraSetup())
  {
    Serial.println("Camera init failed. Halting.");
    while (true) { delay(1000); }
  }
    
  server_Camera.begin();
  Serial.println("Servers started. Waiting for clients...");

  // Start camera streaming task on core 0
  xTaskCreatePinnedToCore(
    loopTask_Camera,
    "loopTask_Camera",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );
}

void loop()
{
  WiFiClient client = server_Cmd.available();

  if (client)
  {
    Serial.printf("Command client connected");
    String recvBuffer;
    lastSensorMs = millis();

    while (client.connected())
    {
      // 1) Read incoming bytes and assemble lines
      while (client.available())
      {
        char c = client.read();
        if (c == ENTER)
        {
          recvBuffer.trim();
          if (recvBuffer.length() > 0)
          {
            handleCommandLine(recvBuffer, client);
          }
          recvBuffer = "";
        }
        else if (c != '\r')
        {
          recvBuffer += c;
        }
      }

      // 2) Periodic sensor report
      unsigned long now = millis();
      if (now - lastSensorMs >= sensorPeriodMs)
      {
        lastSensorMs = now;
        sendSensorData(client);
      }

      delay(5);
    }

    client.stop();
    Serial.println("Command client disconnected.");
    videoSendingFlag = false;
  }

  delay(10);
}
