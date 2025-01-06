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
                                                                                                              
bool imu_connected(bno055_t *i){
    assert(i);
    return i->bno.begin();
}
void imu_set_external_crystal(bno055_t *i){
    assert(i);
    i->bno.setExtCrystalUse(true);
}

data_t imu_euler_x(bno055_t *i){
    assert(i);
    return i->bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
}

#if not DRY
data_t imu_acc_x(bno055_t *i){
    assert(i);
    sensors_event_t a;
    i->bno.getEvent(&a, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return a.acceleration.x;
}

data_t imu_acc_y(bno055_t *i){
    assert(i);
    sensors_event_t a;
    i->bno.getEvent(&a, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return a.acceleration.y;
}

data_t imu_acc_z(bno055_t *i){
    assert(i);
    sensors_event_t a;
    i->bno.getEvent(&a, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return a.acceleration.z;
}

#else

#define imu_acc_getter(axis)                                        \
    data_t imu_acc_##axis(bno055_t const *i) {                      \
        assert(i);                                                  \
        sensors_event_t a;                                          \
        i->bno.getEvent(&a, Adafruit_BNO055::VECTOR_ACCELEROMETER); \
        return a.acceleration.axis;                                 \
    }

imu_acc_getter(x);
imu_acc_getter(y);
imu_acc_getter(z);

#endif
// Generazione delle funzioni


//    _____      _ _ _               _   _             
//   / ____|    | (_) |             | | (_)            
//  | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __  
//  | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \ 
//  | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |
//   \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|
                                                                                                 

void create_calibration(bno055_t *i){
    assert(i);
    uint8_t system, gyro, accel, mag = 0;

    while (!i->bno.isFullyCalibrated()){
        i->bno.getCalibration(&system, &gyro, &accel, &mag);
        serial_d(cal_status_print(system, gyro, accel, mag));
        delay(BNO055_PERIOD_MILLISECS);
    }

    adafruit_bno055_offsets_t newCalib;
    i->bno.getSensorOffsets(newCalib);

    serial_i("Calibration completed");
    //serial_d(offsetsToString(newCalib));

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

String offsets_to_string(const adafruit_bno055_offsets_t &offsets){
    String result = "Offsets:\n";
    result += "Accel X: " + String(offsets.accel_offset_x) + ", ";
    result += "Accel Y: " + String(offsets.accel_offset_y) + ", ";
    result += "Accel Z: " + String(offsets.accel_offset_z) + "\n";
    result += "Gyro X: " + String(offsets.gyro_offset_x) + ", ";
    result += "Gyro Y: " + String(offsets.gyro_offset_y) + ", ";
    result += "Gyro Z: " + String(offsets.gyro_offset_z) + "\n";
    result += "Accel radius " + String(offsets.accel_radius) + ", ";
    result += "Mag radius " + String(offsets.mag_radius);

    return result;
}

String cal_status_print(uint8_t s, uint8_t g, uint8_t a, uint8_t m){
    return "Sys: " + String(s) + "/3, Gyro: " + String(g) + "/3, Acc: " + String(a) + "/3, Mag: " + String(m) + "/3";
}