#include "ADXL337.h"
#include "analogsensor.h"

typedef struct ac337_t {
    analogin_t *x;
    analogin_t *y;
    analogin_t *z;
    data_t k;
}ac337_t;

ac337_t *ac337_new(uint8_t pin_x, uint8_t pin_y, uint8_t pin_z, data_t v_supply,uint8_t bits){
    ac337_t *s = malloc(sizeof(ac337_t));
    if(!s){
        serial_e("Error allocating memory for accelerometer");
        return NULL;
    }
    memset(s,0, sizeof(ac337_t));

    s->k = 6.0 / v_supply;
    s->x = sensor_new(pin_x, v_supply, bits, -v_supply/2);
    s->y = sensor_new(pin_y, v_supply, bits, -v_supply/2);
    s->z = sensor_new(pin_z, v_supply, bits, -v_supply/2);

    if (!s->x || !s->y || !s->z) {
        serial_e("Error allocating memory for sensors");
        
        if (s->x) sensor_free(s->x);
        if (s->y) sensor_free(s->y);
        if (s->z) sensor_free(s->z);
        free(s);
        return NULL;
    }

    return s;
}

void ac337_free(ac337_t *s){
    assert(s);
    sensor_free(s->x);
    sensor_free(s->y);
    sensor_free(s->z);
    free(s);
}

#define ac337_getters(axis)                                 \
    data_t ac337_read_##axis(ac337_t const *s){             \
        assert(s);                                          \
        return sensor_read(s->##axis) * s->k;               \
    }                                                       \
                                                            \
    data_t ac337_filtered_read_##axis(ac337_t const *s){    \
        assert(s);                                          \
        return sensor_read_filtered(s->##axis) * s->k;      \
    }                                                                                               

ac337_getters(x);
ac337_getters(y);
ac337_getters(z);

#ifdef ADXL337_MAIN

#include "ADXL337.h"

ac337_t *s;

void setup() {
    Serial.begin(9600);

    s = ac337_new(A0, A1, A2, 5, 6.0,1024);
    if (!s) {
        serial_e"Allocazione fallita!");
        while (1); 
    }

    serial_i("Test analogin_t Start");
}

void loop() {
    data_t raw_value = sensor_read(ir_sens);
    data_t filtered_value = sensor_read_filtered(ir_sens);

    Serial.print("Raw Value: ");
    Serial.print(raw_value);
    Serial.print(" | Filtered Value: ");
    Serial.println(filtered_value);

    delay(500);
}

#endif
