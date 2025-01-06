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

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

// IMU LIBRARIES
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// CUSTOM LIBRARIES
#include "input/BNO055.h"
#include "input/digitalin.h"
//#include "input/ADXL337.h"
//#include "input/analogsensor.h"
//#include "input/IR_GP2Y0A41.h"
//#include "filters/IIRFilter.h"


//   _____        __ _       _ _   _                 
//  |  __ \      / _(_)     (_) | (_)                
//  | |  | | ___| |_ _ _ __  _| |_ _  ___  _ __  ___ 
//  | |  | |/ _ \  _| | '_ \| | __| |/ _ \| '_ \/ __|
//  | |__| |  __/ | | | | | | | |_| | (_) | | | \__ \
//  |_____/ \___|_| |_|_| |_|_|\__|_|\___/|_| |_|___/

// GLOBAL DEFINITIONS

// Application settings
#define DRY true
#define FLAVOURED_PRINT true
#define FILTERS false

#define AVERAGE_READ 10

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

// Timing
#define BLINK_INTERVAL 250
#define DEBOUNCE_TIME 500
#define BNO055_PERIOD_MILLISECS 5000

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

#if FILTERS
//   ______ _ _ _                
//  |  ____(_) | |               
//  | |__   _| | |_ ___ _ __ ___ 
//  |  __| | | | __/ _ \ '__/ __|
//  | |    | | | ||  __/ |  \__ \
//  |_|    |_|_|\__\___|_|  |___/

/* Butterworth low-pass parameters
// 50 Hz
double a_lp_50Hz[] = {1.000000000000, -3.180638548875, 3.861194348994, -2.112155355111, 0.438265142262};
double b_lp_50Hz[] = {0.000416599204407, 0.001666396817626, 0.002499595226440, 0.001666396817626, 0.000416599204407};

// 30 Hz 
double a_lp_30Hz[] = {1.000000000000, -3.507786207391, 4.640902412687, -2.742652821120, 0.610534807561};
double b_lp_30Hz[] = {0.000062386983548, 0.000249547934194,0.000374321901291, 0.000249547934194, 0.000062386983548};

// 10 Hz
double a_lp_10Hz[] = {1.000000000000, -3.835825540647, 5.520819136622, -3.533535219463, 0.848555999266};
double b_lp_10Hz[] = {0.000000898486146, 0.000003593944586, 0.000005390916878, 0.000003593944586, 0.000000898486146};
*/
#endif 
#endif
