#include "include/defines.h"

// BNO055 sensor
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//bool t = true;

bno055_t *imu;



// debugButton
unsigned long debugButtonLastDebounceTime = 0;

// blinkLedFastNonBlocking variables
static unsigned long blinkLedFastNonBlockingPreviousMillis = 0;
bool blinkLedFastNonBlockingState = LOW;

// clubMovement thresholds
const float baseYThreshold = 9.0; // Base position (club down)
const float upZThreshold = -9.0;  // Club raised (up)
const float tolerance = 2.0;      // Allow ±2 variation

// Initial orientation reference
float initialYaw = 0.0;         // Initial yaw (heading) reference
bool isYawReferenceSet = false; // Flag to ensure it's set only once


void buttonISR(){
  // TODO: check if it works
  if(digital_new_status(imu, millis()))
    initialYaw = imu_euler_x(imu);
}

void setup(void){

  // pins
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);// Button
  pinMode(LED_PIN, OUTPUT);          // Led
  pinMode(HALL_SENSOR_PIN, INPUT);   // Hall sensor

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
  imu = imu_new(10, 18,19,true);

  while (!imu_connected(imu)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(2000);
  }

  initialYaw = imu_euler_x(imu);
  serial_i(String("Yaw: " + initialYaw));

  // end of setup
  digitalWrite(LED_PIN, HIGH);
}

#if 1


void loop(void){


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

  // Button to recalibrate reference

  // Process sensor events

  // Detect movement
  detectClubMovement(imu);

  // Hit the ball
  detectBallHit();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void detectClubMovement(bno055_t *imu){
  // State variables
  static bool isUp = false; // Tracks if the club is currently up

  float accelY = imu_acc_y(imu);
  float accelZ = imu_acc_z(imu);

  // Check if club is in base position
  bool atBase = (accelY >= (baseYThreshold - tolerance) && accelY <= (baseYThreshold + tolerance));
  // Check if club is in up position
  bool atUp = (accelZ >= (upZThreshold - tolerance) && accelZ <= (upZThreshold + tolerance));

  // Determine motion
  if (!isUp && atUp)
  {
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

void detectBallHit()
{
  // Detect ball hit using the hall sensor
  uint8_t hallSensor = digitalRead(HALL_SENSOR_PIN); // Read hall sensor state

  if (hallSensor == LOW) // Ball is hit
  {
    Serial.println("Hit the ball!");

    // TODO Play sound hit

    // Get the current yaw (rotation angle)
    imu::Vector<3> euler = imu->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float currentYaw = euler.x(); // Extract yaw (rotation around Z-axis)

    // Calculate relative yaw based on the initial calibration
    float relativeYaw = currentYaw - initialYaw;

    // Normalize yaw to handle wrap-around (0–360 degrees)
    if (relativeYaw < -180)
      relativeYaw += 360;
    if (relativeYaw > 180)
      relativeYaw -= 360;

    // Print yaw angle for debugging
    Serial.print("Relative Yaw: ");
    Serial.println(relativeYaw);

    // Determine the direction of the ball based on yaw
    if (relativeYaw > 10.0) // Right threshold
    {
      Serial.println("Ball will go RIGHT!");
    }
    else if (relativeYaw < -10.0) // Left threshold
    {
      Serial.println("Ball will go LEFT!");
    }
    else
    {
      Serial.println("Ball will go STRAIGHT!");
    }

    // Wait for 3 seconds before detecting another hit
    delay(3000);
  }
}

void recalibrateYawReference()
{
  imu::Vector<3> euler = imu->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialYaw = euler.x(); // Reset initial yaw
  Serial.print("Yaw Reference Recalibrated: ");
  Serial.println(initialYaw);
}

// Function to blink LED fast non-blocking
void blinkLedFastNonBlocking()
{
  unsigned long currentMillis = millis();

  if (currentMillis - blinkLedFastNonBlockingPreviousMillis >= 250)
  {
    blinkLedFastNonBlockingPreviousMillis = currentMillis;

    if (blinkLedFastNonBlockingState == LOW)
    {
      blinkLedFastNonBlockingState = HIGH;
    }
    else
    {
      blinkLedFastNonBlockingState = LOW;
    }
    digitalWrite(LED_PIN, blinkLedFastNonBlockingState);
  }
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

#endif