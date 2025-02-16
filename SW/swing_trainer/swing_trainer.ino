#include "defines.h"

/**
 * @brief IMU object for swing
 */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/**
* @brief Timer for LED blink
*/
IntervalTimer myTimer;

/**
 * @brief Debounce time reference for debugging
 */
unsigned long debugButtonLastDebounceTime = 0.0;

/**
 * @brief Yaw, Roll and Pitch reference for trajectory evaluation
 */
float initialYaw = 0.0;         // Initial yaw (heading) reference

float initialRoll = 0.0;        // Initial roll reference

float initialPitch = 0.0;       // Initial pitch reference

/**
 * @brief Interrupt status for hall effect sensor
 */
bool hall_interrupt = false;

/**
 *  @brief Interrupt handler for hall effect sensor
 * */
void hallInterrupt(){
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

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallInterrupt, LOW);

  myTimer.begin(blinkLED, DEBOUNCE_TIME); // 500ms

  #if ARDUINO_BOARD
  Serial.begin(BAUD_ARDUINO);
  #endif

  #if TEENSY_BOARD
  Serial.begin(BAUD_TEENSY);
  #endif

  while (!Serial)
    delay(10);

  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);       
  bno.setMode(OPERATION_MODE_NDOF); 

  if (checkCalibrationSaved())
    loadCalibration(bno);
  else{
    Serial.println("No saved calibration found.");
    IMUcalibration(bno, initialRoll, initialPitch, initialYaw);
  }
}


void loop(void){
  sendData(bno, initialRoll, initialPitch, initialYaw);

  detectClubMovement(bno, initialYaw);

  if (hall_interrupt){
    detectBallHit(bno, initialRoll, initialPitch, initialYaw);
    hall_interrupt = false;
  }

  if (digitalRead(BUTTON_PIN) == LOW && (millis() - debugButtonLastDebounceTime > DEBOUNCE_TIME)){
    debugButtonLastDebounceTime = millis();
    recalibrateYawReference(bno, initialRoll, initialPitch, initialYaw);
  }

  hand_feedback(BUTTON_TRIGGER);

  delay(BNO055_SAMPLERATE_DELAY_MS);


}
