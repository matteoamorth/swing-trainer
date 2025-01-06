#include "include/defines.h"

/**
 * @brief IMU located in the club's head
 */
bno055_t *imu_swing;

/**
 * @brief Reset button located in the club's end
 */
digitalin_t *yaw_button;

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

// BNO055 sensor
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// blinkLedFastNonBlocking variables
static unsigned long blinkPreviousMillis = 0;
bool blinkState = LOW;


void buttonISR(){
  // TODO: check if it works
  if(digital_new_status(yaw_button, millis()))
    initialYaw = imu_euler_x(imu_swing);
}

void hallISR(){
  hall_interrupt = true;
}

void setup(void){

  // pins
  // TODO: check function 
  //pinMode(BUTTON_PIN, INPUT_PULLUP); 
  yaw_button = digital_new(BUTTON_PIN, DEBOUNCE_TIME, true, false);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);// Button

  pinMode(HALL_SENSOR_PIN, INPUT);   // Hall sensor
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), buttonISR, LOW);// Button

  pinMode(LED_PIN, OUTPUT);          // Led
  

  // serial port
  #if ARDUINO_BOARD
  Serial.begin(BAUD_ARDUINO);
  #endif

  #if TEENSY_BOARD
  Serial.begin(BAUD_TEENSY);
  #endif

  while (!Serial)
    delay(10); // wait for serial port to open!


  // imu setup
  imu_swing = imu_new(10, 18,19,true);

  while (!imu_connected(imu_swing)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(2000);
  }

  initialYaw = imu_euler_x(imu_swing);
  serial_i(String("Yaw: " + String(initialYaw)));

  // end of setup
  digitalWrite(LED_PIN, HIGH);

}


void loop(void){

  // interrupt services
  if (hall_interrupt)
    detectBallHit();
  
  // Button to DEBUG accelerometer, gyroscope and linear acceleration data
  /*
    Press the button to print accelerometer, gyroscope and linear acceleration data
    and know the current status of the sensor
  
  // if (digitalRead(BUTTON_PIN) == LOW && (millis() - debugButtonLastDebounceTime > DEBOUNCE_TIME))
  // {
  //   debugButtonLastDebounceTime = millis();

  //   // Process sensor events
  //   sensors_event_t angVelocityData, linearAccelData, accelerometerData;
  //   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  //   // Print data
  //   printEvent(&linearAccelData);
  //   printEvent(&angVelocityData);
  //   printEvent(&accelerometerData);
  // }*/

  
  // Detect movement
  detectClubMovement(imu_swing);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void detectClubMovement(bno055_t *imu_swing){
  // State variables
  static bool isUp = false; // Tracks if the club is currently up

  data_t accelY = imu_acc_y(imu_swing);
  data_t accelZ = imu_acc_z(imu_swing);

  // Check if club is in base position
  bool atBase = (accelY >= (BASE_Y_THRESHOLD - TOLERANCE) && accelY <= (BASE_Y_THRESHOLD + TOLERANCE));
  // Check if club is in up position
  bool atUp = (accelZ >= (UP_Z_THRESHOLD - TOLERANCE) && accelZ <= (UP_Z_THRESHOLD + TOLERANCE));

  // Determine motion
  if (!isUp && atUp){
    // Club is being raised
    serial_i("Club is being raised up!");
    isUp = true; // Update state

    // TODO Play sound up
  }
  else if (isUp && atBase)
  {
    // Club is going down
    serial_i("Club is going down!");
    isUp = false; // Update state

    // TODO Play sound down
  }
}

void detectBallHit(){

  data_t yaw = imu_euler_x(imu_swing);
  yaw = yaw - initialYaw;

  // TODO Play sound hit

  // Normalize yaw to handle wrap-around (0–360 degrees)
  yaw = (yaw < -180) ? (yaw + 360) : (yaw > 180 ? (yaw - 360) : yaw);

  // Detect ball hit using the hall sensor
  Serial.println("Hit the ball!");

  serial_d("Relative Yaw: ");
  serial_d(yaw);

  // Determine the direction of the ball based on yaw
  Serial.println((yaw > 10.0) ? "Ball will go RIGHT!" : (yaw < -10.0) ? "Ball will go LEFT!" : "Ball will go STRAIGHT!");

  delay(2000);

  // reset interrupt
  hall_interrupt = false;
  
}

/*
void recalibrateYawReference()
{
  imu_swing::Vector<3> euler = imu_swing->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialYaw = euler.x(); // Reset initial yaw
  Serial.print("Yaw Reference Recalibrated: ");
  Serial.println(initialYaw);
}
*/
// Function to blink LED fast non-blocking
void blinkLedFastNonBlocking(){
  unsigned long currentMillis = millis();

  if (currentMillis - blinkPreviousMillis < BLINK_INTERVAL)
    return;

  blinkPreviousMillis = currentMillis;
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  
}

void printEvent(sensors_event_t *event){
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else
  {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

