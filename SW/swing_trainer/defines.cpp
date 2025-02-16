#include "defines.h"

void blinkLED(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void detectClubMovement(Adafruit_BNO055 bno, float ref_yaw){
  float ac_y = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).y();
  float ac_z = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z();
  float yaw = bno.getVector(Adafruit_BNO055::VECTOR_EULER).z() - ref_yaw;

  bool atBase = fabs(ac_y - BASE_Y_THRESHOLD) <= TOLERANCE;
  bool atUp = fabs(ac_z - UP_Z_THRESHOLD) <= TOLERANCE;

  // Determine motion
  if (!isUp && atUp){
    Serial.println("UP 1.0");
    isUp = true; 
  } 
  
  if (isUp && atBase){
    Serial.println("DOWN 1.0");
    isUp = false; 
  }

  #if HAPTIC_FEEDBACK
  if (yaw > -100)
    return;

  analogWrite(VIBRATOR, 200);
  delay(100);
  analogWrite(VIBRATOR, 0);
  delay(100);

  #endif
  
}

void detectBallHit(Adafruit_BNO055 bno, float initialRoll, float initialPitch, float initialYaw){

  Serial.println("HIT 1.0");
  sendData(bno, initialRoll, initialPitch, initialYaw);

  float roll = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x() - initialRoll;

  #if HAPTIC_FEEDBACK
  analogWrite(VIBRATOR, 200);
  delay(200);
  analogWrite(VIBRATOR, 0);
  #endif

  roll = fmod(roll + 180, 360) - 180; // Normalize to -180, 180
  Serial.print("ROLL: ");
  Serial.println(roll);
  handleLEDFeedback(roll);

  delay(2000);
  Serial.println("HIT 0");
}

void handleLEDFeedback(float roll) {
  int led = (roll > 7.0) ? LED_RIGHT : (roll < -7.0) ? LED_LEFT : LED_STRAIGHT;
  for (int i = 0; i < 5; i++) {
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(400);
  }
}

void recalibrateYawReference(Adafruit_BNO055& bno, float& initialRoll, float& initialPitch, float& initialYaw){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialRoll = euler.x();
  initialPitch = euler.y();
  initialYaw = euler.z(); // Reset initial yaw
  Serial.print("Reference Recalibrated: ");
  Serial.print(initialRoll);
  Serial.print(initialPitch);
  Serial.println(initialYaw);
}

void saveCalibration(Adafruit_BNO055 bno){
  uint8_t calibrationData[CALIBRATION_SIZE];          
  bno.getSensorOffsets(calibrationData); 
  for (int i = 0; i < 22; i++){
    EEPROM.write(EEPROM_ADDRESS + i, calibrationData[i]); // Write each byte
  }
  EEPROM.write(EEPROM_ADDRESS + 22, 1); // Write a flag to indicate data saved
  Serial.println("Calibration Saved");
}

void loadCalibration(Adafruit_BNO055 bno){
  uint8_t calibrationData[22];
  for (int i = 0; i < 22; i++)
  {
    calibrationData[i] = EEPROM.read(EEPROM_ADDRESS + i); // Read each byte
  }
  bno.setSensorOffsets(calibrationData); // Apply offsets
  Serial.println("Calibration Loaded");
}

bool checkCalibrationSaved(){
  return EEPROM.read(EEPROM_ADDRESS + 22) == 1; 
}

void IMUcalibration(Adafruit_BNO055& bno, float& initialRoll, float& initialPitch, float& initialYaw){
  uint8_t system, gyro, accel, mag = 0;
  while ((system & gyro & accel & mag) != 3){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    printCalibration(system, gyro, accel, mag);
  }

  saveCalibration(bno);
  recalibrateYawReference(bno, initialRoll, initialPitch, initialYaw);
}

void clearEEPROM(){
  for (int i = 0; i < EEPROM.length(); i++)
    EEPROM.write(i, 0); 
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

void sendData(Adafruit_BNO055 bno, float const initialRoll, float const initialPitch, float const initialYaw){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  

  Serial.print("Roll ");
  Serial.println(euler.x() - initialRoll);

  Serial.print("Pitch ");
  Serial.println(euler.y() - initialPitch);

  Serial.print("Yaw ");
  Serial.println(euler.z() - initialYaw);

  Serial.print("Accel ");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.println(accel.z());
}

void hand_feedback(int pin){
  if (digitalRead(pin) == LOW) 
    return;

  float ribbon_v = analogRead(RIBBON_SENSOR) - RIBBON_CENTRAL_VALUE;
  float threshold = RIBBON_CENTRAL_VALUE * RIBBON_TOLERANCE;

  digitalWrite(LED_UP, ribbon_v > threshold);
  digitalWrite(LED_CENTER, abs(ribbon_v) <= threshold);
  digitalWrite(LED_DOWN, ribbon_v < -threshold);
  
}