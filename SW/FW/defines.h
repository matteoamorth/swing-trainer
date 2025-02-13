#ifndef DEFINES_H
#define DEFINES_H


// COMMON TYPES
typedef float data_t;


//   _____            _           _           
//  |_   _|          | |         | |          
//    | |  _ __   ___| |_   _  __| | ___  ___ 
//    | | | '_ \ / __| | | | |/ _` |/ _ \/ __|
//   _| |_| | | | (__| | |_| | (_| |  __/\__ \
//  |_____|_| |_|\___|_|\__,_|\__,_|\___||___/
                                           
                                        
// COMMON LIBRARIES
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <EEPROM.h>
#include <Wire.h>

// IMU LIBRARIES
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



//   _____        __ _       _ _   _                 
//  |  __ \      / _(_)     (_) | (_)                
//  | |  | | ___| |_ _ _ __  _| |_ _  ___  _ __  ___ 
//  | |  | |/ _ \  _| | '_ \| | __| |/ _ \| '_ \/ __|
//  | |__| |  __/ | | | | | | | |_| | (_) | | | \__ \
//  |_____/ \___|_| |_|_| |_|_|\__|_|\___/|_| |_|___/

// GLOBAL DEFINITIONS

// Application settings
#define FLAVOURED_PRINT false


// Thresholds
#define BASE_Y_THRESHOLD 9.0    // Base position (club down)
#define UP_Z_THRESHOLD -9.0     // Club raised (up)
#define TOLERANCE 2.0           // Allow ±2 variation

// Board settings
#define ARDUINO_BOARD 0
#define TEENSY_BOARD 1
#define BAUD_ARDUINO 9800
#define BAUD_TEENSY 115200
#define EE_ADDR 0
#define EEPROM_ADDRESS EE_ADDR

// Pin definition
#define LED_PIN 3
#define BUTTON_PIN 2
#define HALL_SENSOR_PIN 4
#define BUTTON_TRIGGER 5
#define LED_UP 7
#define LED_CENTER 8
#define LED_DOWN 12
#define LED_LEFT 10
#define LED_STRAIGHT 11
#define LED_RIGHT 9
#define RIBBON_SENSOR A0
#define VIBRATOR 6

// Timing
#define BLINK_INTERVAL 250
#define DEBOUNCE_TIME 500
#define BNO055_SAMPLERATE_DELAY_MS 100

// Ribbon settings
#define RIBBON_CENTRAL_VALUE 700
#define RIBBON_TOLERANCE 0.1

// Compatibility IDE
#ifndef __ARDUINO_IDE
#define __ARDUINO_IDE
#define INPUT_PULLUP 0x2
#define OUTPUT 0x1
#define INPUT 0x0
#define HIGH 0x1
#define LOW 0x0
#endif


/*
//   _____      _       _    
//  |  __ \    (_)     | |   
//  | |__) | __ _ _ __ | |_  
//  |  ___/ '__| | '_ \| __| 
//  | |   | |  | | | | | |_  
//  |_|   |_|  |_|_| |_|\__|             
*/
#if not FLAVOURED_PRINT
#define serial_e(msg) Serial.println(msg)
#define serial_w(msg) Serial.println(msg)
#define serial_i(msg) Serial.println(msg)
#define serial_d(msg) Serial.println(msg)

#else

//Regular text
#define BLK "\e[0;30m"
#define RED "\e[0;31m"
#define GRN "\e[0;32m"
#define YEL "\e[0;33m"
#define BLU "\e[0;34m"
#define MAG "\e[0;35m"
#define CYN "\e[0;36m"
#define WHT "\e[0;37m"
#define CRESET "\e[0m"

// MACRO 
#define serial_e(msg)    Serial.print(RED); Serial.print(millis()/500); Serial.print(" *** ERROR: "); Serial.println(msg); Serial.print(CRESET)
#define serial_w(msg)    Serial.print(YEL); Serial.print(millis()/500); Serial.print(" *** WARNING: "); Serial.println(msg); Serial.print(CRESET)
#define serial_i(msg)    Serial.print(GRN); Serial.print(millis()/500); Serial.print(" *** INFO: "); Serial.println(msg); Serial.print(CRESET)
#define serial_d(msg)    Serial.print(BLU); Serial.print(millis()/500); Serial.print(" *** DEBUG: "); Serial.println(msg); Serial.print(CRESET)
#endif

// function declarations
void detectClubMovement(Adafruit_BNO055 bno, float ref_yaw);
void detectBallHit(Adafruit_BNO055 bno);
void recalibrateYawReference();
void saveCalibration();
void loadCalibration();
bool checkCalibrationSaved();
void IMUcalibration(Adafruit_BNO055 bno);
void clearEEPROM();
void printCalibration(uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag);
void sendData(Adafruit_BNO055 bno);
void hand_feedback(int pin);

#endif
