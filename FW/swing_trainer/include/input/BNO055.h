#ifndef BNO055_H
#define BNO055_H

#include "../defines.h"

#define BNO055_SAMPLERATE_DELAY_MS 100

typedef struct bno055_t bno055_t;

bno055_t *imu_new(uint8_t _id, uint8_t pin_sda, uint8_t pin_scl, bool calibrate);
bool imu_connected(bno055_t *i);
void imu_set_external_crystal(bno055_t *i);
float imu_euler_x(bno055_t *i);

float imu_acc_x(bno055_t const *i);
float imu_acc_y(bno055_t const *i);
float imu_acc_z(bno055_t const *i);

void create_calibration(bno055_t *i);
void magnetometer_calibration(bno055_t *i);
bool check_calibration(bno055_t *i);
void load_calibration(bno055_t *i);
void bno055_reset_calibration();

String offsetsToString(const adafruit_bno055_offsets_t &offsets);
String cal_status_print(uint8_t s, uint8_t g, uint8_t a, uint8_t m);


#endif
