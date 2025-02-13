#include "defines.h"

/**
 * @brief Interrupt status for hall effect sensor
 */
bool hall_interrupt = false;

/**
 * @brief Debounce time reference for debugging
 */
unsigned long debugButtonLastDebounceTime = 0.0;

/**
 * @brief Yaw reference for trajectory evaluation
 */
float initialYaw = 0.0;         // Initial yaw (heading) reference

/**
 * @brief IMU object for swing
 */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);




void buttonISR(){
  // TODO: check if it works
  if(digital_new_status(yaw_button, millis()))
    initialYaw = imu_euler_x(imu_swing);
}

void hitDetected(){
  if(digitalRead(BUTTON_TRIGGER) == LOW)
    hall_interrupt = true;
}

void setup(void){

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button
  pinMode(LED_PIN, OUTPUT);          // Led
  pinMode(HALL_SENSOR_PIN, INPUT);   // Hall sensor

  // high club part
  pinMode(BUTTON_TRIGGER, INPUT_PULLUP);
  pinMode(RIBBON_SENSOR, INPUT);
  pinMode(LED_UP, OUTPUT);
  pinMode(LED_CENTER, OUTPUT);
  pinMode(LED_DOWN, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_STRAIGHT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(VIBRATOR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hitDetected, LOW);

  

  // serial port
  #if ARDUINO_BOARD
  Serial.begin(BAUD_ARDUINO);
  #endif

  #if TEENSY_BOARD
  Serial.begin(BAUD_TEENSY);
  #endif

  while (!Serial)
    delay(10); // wait for serial port to open!

  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);       // Use external crystal for better accuracy
  bno.setMode(OPERATION_MODE_NDOF); // Set sensor mode to NDOF

  // Load calibration from EEPROM
  if (checkCalibrationSaved())
    loadCalibration();
  else{
    Serial.println("No saved calibration found.");
    IMUcalibration(bno);
  }
}


void loop(void){
  sendData(bno);

  detectClubMovement(bno, initialYaw);

  // interrupt services
  if (hall_interrupt)
    detectBallHit();

  // Button to recalibrate reference
  if (digitalRead(BUTTON_PIN) == LOW && (millis() - debugButtonLastDebounceTime > DEBOUNCE_TIME)){
    debugButtonLastDebounceTime = millis();
    recalibrateYawReference();
  }

  hand_feedback(BUTTON_TRIGGER);

  delay(BNO055_SAMPLERATE_DELAY_MS);


}
