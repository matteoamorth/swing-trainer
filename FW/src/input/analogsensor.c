#include "analogsensor.h"

typedef struct analogin_t{
    uint8_t pin;
    data_t k;
    data_t q;
}analogin_t;

//   _____                 _   _
//  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
//  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
//  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/

analogin_t *sensor_new(uint8_t _pin, uint8_t v_supply, uint8_t bits, data_t offset){
    analogin_t *s = malloc(sizeof(analogin_t));
    if(!s){
        serial_e("Error allocating memory for analog sensor");
        return NULL;
    }

    memset(s,0,sizeof(analogin_t));

    s->pin = _pin;
    s->k = (data_t) v_supply / bits;
    s->q = offset;

    return s;
}

void sensor_free(analogin_t *s){
    assert(s);
    free(s);
}

data_t sensor_read(analogin_t const *s){
    assert(s);
    return s->k * analogRead(s->pin) + s->q; 
}

data_t sensor_read_filtered(analogin_t const *s){
    assert(s);
    data_t m = 0;

    for (uint8_t i = 0; i < AVERAGE_READ; i++){
        m += IR_read(s);
    }
    
    return m / AVERAGE_READ;
}

#ifdef ANALOG_SENSOR_MAIN

#include "analogsensor.h"

analogin_t *s;

void setup() {
    Serial.begin(9600);

    s = sensor_new(A0, 5, 1024);
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
