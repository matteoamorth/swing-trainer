#include "BNO055.h"

typedef struct bno055_t{
    uint8_t sda;
    uint8_t scl;
    Adafruit_BNO055 *bno;
    
}bno055_t;

bno055_t *imu_new(uint8_t _id, uint8_t pin_sda, uint8_t pin_scl, bool calibrate){
    bno055_t *i = malloc(sizeof(bno055_t));
    if(!i){
        serial_e("Can't allocate memory for IMU!");
        return NULL;
    }   

    memset(i,0, sizeof(bno055_t));

    i->bno = Adafruit_BNO055(id);
    i->sda = pin_sda;
    i->scl = pin_scl;
    
    if(!calibrate)
        return i;

    load_calibration();

}



void create_calibration(bno055_t *i){
    assert(i);
    assert(i->bno);

    while (!i->bno->isFullyCalibrated()){
        displayCalStatus();
        serial_d("");
        delay(BNO055_PERIOD_MILLISECS);
    }

    adafruit_bno055_offsets_t newCalib;
    i->bno->getSensorOffsets(newCalib);

    serial_i("Calibration completed");
    displaySensorOffsets(newCalib);

    EEPROM.put(EE_ADDR, bnoID);
    EEPROM.put(EE_ADDR + sizeof(long), newCalib);

}

void magnetometer_calibration(bno055_t *i){
    assert(i);
    assert(i->bno);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
 
    while (mag != 3) {
    
    i->bno->getCalibration(&system, &gyro, &accel, &mag);
    if(display_BNO055_info){
      
      displayCalStatus();
      Serial.println("");
    }
  }
}

bool check_calibration(bno055_t *i){
    assert(i);
    assert(i->bno);

    long eeBnoID;
    sensor_t sensor;

    i->bno->getSensor(&sensor);
    EEPROM.get(EE_ADDR, eeBnoID);

    return eeBnoID == sensor.sensor_id;
}

void load_calibration(bno055_t *i){
    assert(i);
    assert(i->bno);

    if(!check_calibration(i)){
        serial_w("No calibration data found");
        create_calibration(i);
        return; 
    }

    serial_i("Calibration found");
    long calibrationData;
    EEPROM.get((EE_ADDR + sizeof(long)), calibrationData);
    i->bno->setSensorOffsets(calibrationData);

    // always calibrate magnetometer
    magnetometer_calibration(i);

}

void bno055_reset_calibration(){
    EEPROM.put(EE_ADDR, 0);
    EEPROM.put(sizeof(long), 0);
    serial_d("EEprom reset");
}