#include "../../include/input/BNO055.h"

typedef struct bno055_t{
    uint8_t bnoID;
    uint8_t sda;
    uint8_t scl;
    Adafruit_BNO055 bno;
    
}bno055_t;

bno055_t *imu_new(uint8_t _id, uint8_t pin_sda, uint8_t pin_scl, bool calibrate){
    bno055_t *i = malloc(sizeof(bno055_t));
    if(!i){
        serial_e("Can't allocate memory for IMU!");
        return NULL;
    }   

    memset(i,0, sizeof(bno055_t));

    i->bnoID = _id;
    i->bno = Adafruit_BNO055(_id, 0x28, &Wire);
    i->sda = pin_sda;
    i->scl = pin_scl;
    
    if(!calibrate)
        return i;

    load_calibration(i);

}

//   _____ __  __ _    _    _____      _   _                
//  |_   _|  \/  | |  | |  / ____|    | | | |               
//    | | | \  / | |  | | | (___   ___| |_| |_ ___ _ __ ___ 
//    | | | |\/| | |  | |  \___ \ / _ \ __| __/ _ \ '__/ __|
//   _| |_| |  | | |__| |  ____) |  __/ |_| ||  __/ |  \__ \
//  |_____|_|  |_|\____/  |_____/ \___|\__|\__\___|_|  |___/
                                                                                                              
bool imu_connected(bno055_t *i)
    return i->bno.begin();

void imu_set_external_crystal(bno055_t *i)
    i->bno.setExtCrystalUse(true);

//    _____      _ _ _               _   _             
//   / ____|    | (_) |             | | (_)            
//  | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __  
//  | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \ 
//  | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |
//   \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|
                                                                                                 

void create_calibration(bno055_t *i){
    assert(i);

    while (!i->bno.isFullyCalibrated()){
        //displayCalStatus();
        serial_d("");
        delay(BNO055_PERIOD_MILLISECS);
    }

    adafruit_bno055_offsets_t newCalib;
    i->bno.getSensorOffsets(newCalib);

    serial_i("Calibration completed");
    serial_d(newCalib);

    EEPROM.put(EE_ADDR, i->bnoID);
    EEPROM.put(EE_ADDR + sizeof(long), newCalib);

}

void magnetometer_calibration(bno055_t *i){
    assert(i);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
 
    while (mag != 3) {
    
    i->bno.getCalibration(&system, &gyro, &accel, &mag);
    serial_i("");
    
  }
}

bool check_calibration(bno055_t *i){
    assert(i);

    long eeBnoID;
    sensor_t sensor;

    i->bno.getSensor(&sensor);
    EEPROM.get(EE_ADDR, eeBnoID);

    return eeBnoID == sensor.sensor_id;
}

void load_calibration(bno055_t *i){
    assert(i);

    if(!check_calibration(i)){
        serial_w("No calibration data found");
        create_calibration(i);
        return; 
    }

    serial_i("Calibration found");
    long calibrationData;
    EEPROM.get((EE_ADDR + sizeof(long)), calibrationData);
    i->bno.setSensorOffsets(calibrationData);

    // always calibrate magnetometer
    magnetometer_calibration(i);

}

void bno055_reset_calibration(){
    EEPROM.put(EE_ADDR, 0);
    EEPROM.put(sizeof(long), 0);
    serial_d("EEprom reset");
}