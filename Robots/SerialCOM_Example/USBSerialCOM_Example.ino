


/// Globals

// Serial
const uint32_t serialBaudRate = 1000000; //19200 default for ide serial monitor; // original default 1500000

// Motors and Servos
byte const n_servos = 4;
byte Actions[ n_servos ] = { 127, 127, 127, 127 };

// Camera
JpegFrame_t frame;




void setup() {
    
  // Serial USB Connection
  Serial.begin(serialBaudRate); 
  Serial.setTimeout(0.1);

}


/// Loop Start
void loop() {

  // Read Sensors
  
  // Send Sensors
  sendSerialIMUAndServoFeedbacks();
  
  // Receive Serial Actions / Servo or Motor Signals
  receiveSerialActions();

  // Apply Servo Signals
  
  // Get Image

  // Send Image
  sendImgSerial();

}




// -- Serial Receive --
void receiveSerialActions() {
  if (Serial.available() >= n_servos)
  {
    for (uint8_t i = 0; i < n_servos; i++) {
      Actions[i] = Serial.read();
    }
    // there might be a lot of bytes coming in, this clears the buffer by reading the remaining bytes
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

// -- Serial SEND --
void sendSerialIMUAndServoFeedbacks() {

  // Start Codons
  Serial.write(202u);
  Serial.write(202u);

  // Send IMU Sensors (9 IMU + 1 Temperature if available)
  for (byte i = 0; i < 10; i++) {
    byte* floatbytes = reinterpret_cast<byte*>(IMUData[i]);
    Serial.write(floatbytes, 4);
  }

  // Send Servo Feedbacks
  for (byte i = 0; i < n_servos; i++) {
      Serial.write( ServoFeedbacks[i] );
  }

  // Send Height Distance Sensor
  uint32_t distbytes = (uint32_t)dist;
  Serial.write((byte *) &distbytes, 4);

  // Stop Codons
  Serial.write(203u);
  Serial.write(203u);

}

// -- Serial Image SEND --
void sendImgSerial() {
  // Start Codons
  Serial.write(222u);
  Serial.write(222u);
  // Size for additional check instead of checksum
  long int lVal = frame.size;
  byte lBuffer[] = {
    byte(lVal & 0xff),
    byte(lVal >> 8 & 0xff),
    byte(lVal >> 16 & 0xff),
    byte(lVal >> 24 & 0xff)
  };
  Serial.write(lBuffer, 4);
  // Image
  for (uint32_t i = 0; i < frame.size; i++) 
  {
    Serial.write( frame.buf[i] );
  }
  // Stop Codons
  Serial.write(223u);
  Serial.write(223u);
}
