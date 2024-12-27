#include "include/defines.h"

// BNO055 sensor
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool t = true;
bno055_t *accelerometer = imu_new(10, 18,19,true);


// debugButton
const int debugButtonDebounceDelay = 500;
unsigned long debugButtonLastDebounceTime = 0;

// IMU calibration status
bool isIMUCalibrated = false;

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


void setup(void)
{
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button
  pinMode(LED_PIN, OUTPUT);          // Led
  pinMode(HALL_SENSOR_PIN, INPUT);   // Hall sensor

  #if ARDUINO_BOARD
  Serial.begin(BAUD_ARDUINO);
  #endif

  #if TEENSY_BOARD
  Serial.begin(BAUD_TEENSY);
  #endif

  while (!Serial)
    delay(10); // wait for serial port to open!

  // Initialize BNO055 sensor
  if (!imu_connected(accelerometer)){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  imu_set_external_crystal(accelerometer); // Use external crystal for better accuracy

  // To clear EEPROM uncomment the following line
  // clearEEPROM();
#if 0
  // Load calibration from EEPROM
  if (checkCalibrationSaved())
  {
    loadCalibration();
    isIMUCalibrated = true;
  }
  else
  {
    isIMUCalibrated = false;
    Serial.println("No saved calibration found.");
  }

  delay(1000);
  Serial.println("Ready!");
}

void loop(void)
{
  while (!isIMUCalibrated) // Calibrate IMU
  {
    blinkLedFastNonBlocking();
    uint8_t system, gyro, accel, mag = 0;
    accelerometer->bno.getCalibration(&system, &gyro, &accel, &mag);

    printCalibration(system, gyro, accel, mag);

    if (system == 3 && gyro == 3 && accel == 3 && mag == 3)
    {
      saveCalibration();

      // Capture initial yaw orientation using magnetometer
      imu::Vector<3> euler = accelerometer->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      initialYaw = euler.x(); // 'x' corresponds to yaw in VECTOR_EULER mode
      isYawReferenceSet = true;

      Serial.print("Initial Yaw: ");
      Serial.println(initialYaw);

      isIMUCalibrated = true;
      digitalWrite(LED_PIN, HIGH); // Set LED stable
    }
  }

  // Button to DEBUG accelerometer, gyroscope and linear acceleration data
  /*
    Press the button to print accelerometer, gyroscope and linear acceleration data
    and know the current status of the sensor
  */
  // if (digitalRead(BUTTON_PIN) == LOW && (millis() - debugButtonLastDebounceTime > debugButtonDebounceDelay))
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
  // }

  // Button to recalibrate reference
  if (digitalRead(BUTTON_PIN) == LOW && (millis() - debugButtonLastDebounceTime > debugButtonDebounceDelay))
  {
    debugButtonLastDebounceTime = millis();
    recalibrateYawReference();
  }

  // Process sensor events
  sensors_event_t accelerometerData;
  accelerometer->bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Extract Y and Z axes from accelerometer
  float accelY = accelerometerData.acceleration.y;
  float accelZ = accelerometerData.acceleration.z;

  // Detect movement
  detectClubMovement(accelY, accelZ);

  // Hit the ball
  detectBallHit();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void detectClubMovement(float accelY, float accelZ)
{
  // State variables
  static bool isUp = false; // Tracks if the club is currently up

  // Check if club is in base position
  bool atBase = (accelY >= (baseYThreshold - tolerance) && accelY <= (baseYThreshold + tolerance));
  // Check if club is in up position
  bool atUp = (accelZ >= (upZThreshold - tolerance) && accelZ <= (upZThreshold + tolerance));

  // Determine motion
  if (!isUp && atUp)
  {
    // Club is being raised
    Serial.println("Club is being raised up!");
    isUp = true; // Update state

    // TODO Play sound up
  }
  else if (isUp && atBase)
  {
    // Club is going down
    Serial.println("Club is going down!");
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
    imu::Vector<3> euler = accelerometer->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
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
  imu::Vector<3> euler = accelerometer->bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialYaw = euler.x(); // Reset initial yaw
  Serial.print("Yaw Reference Recalibrated: ");
  Serial.println(initialYaw);
}

// Function to save calibration data to EEPROM
void saveCalibration()
{
  uint8_t calibrationData[22];           // Calibration data size is 22 bytes
  accelerometer->bno.getSensorOffsets(calibrationData); // Get offsets
  for (int i = 0; i < 22; i++)
  {
    EEPROM.write(EEPROM_ADDRESS + i, calibrationData[i]); // Write each byte
  }
  EEPROM.write(EEPROM_ADDRESS + 22, 1); // Write a flag to indicate data saved
  Serial.println("Calibration Saved");
}

// Function to load calibration data from EEPROM
void loadCalibration()
{
  uint8_t calibrationData[22];
  for (int i = 0; i < 22; i++)
  {
    calibrationData[i] = EEPROM.read(EEPROM_ADDRESS + i); // Read each byte
  }
  accelerometer->bno.setSensorOffsets(calibrationData); // Apply offsets
  Serial.println("Calibration Loaded");
}

// Function to check if calibration is saved in EEPROM
bool checkCalibrationSaved()
{
  return EEPROM.read(EEPROM_ADDRESS + 22) == 1; // Check flag byte
}

// Function to clear EEPROM
void clearEEPROM()
{
  // Loop through all EEPROM addresses
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0); // Write 0 to each address
  }
  Serial.println("EEPROM cleared!");
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

void printCalibration(uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag)
{
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
}

void printEvent(sensors_event_t *event)
{
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