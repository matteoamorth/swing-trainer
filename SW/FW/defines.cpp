#include "defines.h"
void detectClubMovement(Adafruit_BNO055 bno, float ref_yaw){
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Extract Y and Z axes from accelerometer
  float accelY = accelerometerData.acceleration.y;
  float accelZ = accelerometerData.acceleration.z;
  
  static bool isUp = false;
  // Check if club is in base position
  bool atBase = (accelY >= (BASE_Y_THRESHOLD - TOLERANCE) && accelY <= (BASE_Y_THRESHOLD + TOLERANCE));
  // Check if club is in up position
  bool atUp = (accelZ >= (UP_Z_THRESHOLD - TOLERANCE) && accelZ <= (UP_Z_THRESHOLD + TOLERANCE));

  // Determine motion
  if (!isUp && atUp)
  {
    // Club is being raised
    Serial.println("UP 1.0");
    isUp = true; // Update state
  }
  else if (isUp && atBase)
  {
    // Club is going down
    Serial.println("DOWN 1.0");
    isUp = false; // Update state
  }

  float yaw = euler.z() - initialYaw; // Yaw (angle of rotation about z-axis)

  if (yaw < -100){
      analogWrite(VIBRATOR, 200);
      delay(100);
      analogWrite(VIBRATOR, 0);
      delay(100);
    }
    else{
      analogWrite(VIBRATOR, 0);
    }
}

void detectBallHit(Adafruit_BNO055 bno){

  Serial.println("HIT 1.0");
  sendData(bno);

  hallInterrupt = false;

  // feedback about roll X
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // here vibration motor (or managed by message via UART?)
  if (HAPTIC_FEEDBACK)
  {
    analogWrite(VIBRATOR, 200);
    delay(200);
    analogWrite(VIBRATOR, 0);
  }

  float roll = euler.x() - initialRoll;
  roll = roll > 180 ? roll - 360 : (roll < -180 ? roll + 360 : roll);
  Serial.print("ROLL: ");
  Serial.println(roll);
  if (roll > 7.0) // Right threshold
  {
    digitalWrite(LED_RIGHT, HIGH);
    delay(200);
    digitalWrite(LED_RIGHT, LOW);
    delay(200);
    digitalWrite(LED_RIGHT, HIGH);
    delay(200);
    digitalWrite(LED_RIGHT, LOW);
    delay(200);
    digitalWrite(LED_RIGHT, HIGH);
    delay(200);
    digitalWrite(LED_RIGHT, LOW);
  }
  else if (roll < -7.0) // Left threshold
  {
    digitalWrite(LED_LEFT, HIGH);
    delay(1000);
    digitalWrite(LED_LEFT, LOW);
    delay(200);
    digitalWrite(LED_LEFT, HIGH);
    delay(200);
    digitalWrite(LED_LEFT, LOW);
    delay(200);
    digitalWrite(LED_LEFT, HIGH);
    delay(200);
    digitalWrite(LED_LEFT, LOW);
  }
  else
  {
    digitalWrite(LED_STRAIGHT, HIGH);
    delay(200);
    digitalWrite(LED_STRAIGHT, LOW);
    delay(200);
    digitalWrite(LED_STRAIGHT, HIGH);
    delay(200);
    digitalWrite(LED_STRAIGHT, LOW);
    delay(200);
    digitalWrite(LED_STRAIGHT, HIGH);
    delay(200);
    digitalWrite(LED_STRAIGHT, LOW);
  }

  delay(2000);
  Serial.println("HIT 0");
}

void recalibrateYawReference(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialRoll = euler.x();
  initialPitch = euler.y();
  initialYaw = euler.z(); // Reset initial yaw
  Serial.print("Reference Recalibrated: ");
  Serial.print(initialRoll);
  Serial.print(initialPitch);
  Serial.println(initialYaw);
}

// Function to save calibration data to EEPROM
void saveCalibration(){
  uint8_t calibrationData[22];           // Calibration data size is 22 bytes
  bno.getSensorOffsets(calibrationData); // Get offsets
  for (int i = 0; i < 22; i++)
  {
    EEPROM.write(EEPROM_ADDRESS + i, calibrationData[i]); // Write each byte
  }
  EEPROM.write(EEPROM_ADDRESS + 22, 1); // Write a flag to indicate data saved
  Serial.println("Calibration Saved");
}

// Function to load calibration data from EEPROM
void loadCalibration(){
  uint8_t calibrationData[22];
  for (int i = 0; i < 22; i++)
  {
    calibrationData[i] = EEPROM.read(EEPROM_ADDRESS + i); // Read each byte
  }
  bno.setSensorOffsets(calibrationData); // Apply offsets
  Serial.println("Calibration Loaded");
}

// Function to check if calibration is saved in EEPROM
bool checkCalibrationSaved(){
  return EEPROM.read(EEPROM_ADDRESS + 22) == 1; // Check flag byte
}

// Function to calibrate IMU
void IMUcalibration(Adafruit_BNO055 bno){
  while (!isIMUCalibrated) // Calibrate IMU
  {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    printCalibration(system, gyro, accel, mag);

    if (system == 3 && gyro == 3 && accel == 3 && mag == 3)
    {
      saveCalibration();

      // Capture initial yaw orientation using magnetometer
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      initialRoll = euler.x();
      initialPitch = euler.y();
      initialYaw = euler.z(); // 'x' corresponds to yaw in VECTOR_EULER mode
      isYawReferenceSet = true;

      Serial.print("Initial Yaw: ");
      Serial.println(initialYaw);

      isIMUCalibrated = true;
      digitalWrite(LED_PIN, HIGH); // Set LED stable
    }
  }
}

// Function to clear EEPROM
void clearEEPROM(){
  // Loop through all EEPROM addresses
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0); // Write 0 to each address
  }
  Serial.println("EEPROM cleared!");
}

void printCalibration(uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag){
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
}

void sendData(Adafruit_BNO055 bno){
  // Fetch orientation data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll = euler.x() - initialRoll;   // Roll (angle of rotation about x-axis)
  float pitch = euler.y() - initialPitch; // Pitch (angle of rotation about y-axis)
  float yaw = euler.z() - initialYaw;     // Yaw (angle of rotation about z-axis)

  // Fetch acceleration data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float ax = accel.x(); // Acceleration in x-axis
  float ay = accel.y(); // Acceleration in y-axis
  float az = accel.z(); // Acceleration in z-axis

  // Output results
  Serial.print("Roll ");
  Serial.println(roll);
  Serial.print("Pitch ");
  Serial.println(pitch);
  Serial.print("Yaw ");
  Serial.println(yaw);
  Serial.print("Accel ");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.println(az);
}

void hand_feedback(int pin){
  if (digitalRead(pin) == HIGH) {
    float ribbon_v = analogRead(RIBBON_SENSOR) - RIBBON_CENTRAL_VALUE;
    float threshold = RIBBON_CENTRAL_VALUE * RIBBON_TOLERANCE;

    bool ledStates[3] = {
        ribbon_v > threshold,     // LED_UP
        abs(ribbon_v) <= threshold, // LED_CENTER
        ribbon_v < -threshold     // LED_DOWN
    };

    digitalWrite(LED_UP, ledStates[0]);
    digitalWrite(LED_CENTER, ledStates[1]);
    digitalWrite(LED_DOWN, ledStates[2]);
  } 
}