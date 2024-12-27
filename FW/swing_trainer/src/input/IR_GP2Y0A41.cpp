#include "../../include/input/IR_GP2Y0A41.h"

//   _____ _____     _____                            
//  |_   _|  __ \   / ____|                           
//    | | | |__) | | (___   ___ _ __  ___  ___  _ __  
//    | | |  _  /   \___ \ / _ \ '_ \/ __|/ _ \| '__| 
//   _| |_| | \ \   ____) |  __/ | | \__ \ (_) | |    
//  |_____|_|  \_\ |_____/ \___|_| |_|___/\___/|_|                                      
// GP2Y0A41 IR sensor

typedef struct IR_GP2Y0A41_t {
    uint8_t pin;   // Analog pin number connected to the sensor. 
    data_t k;      // Costant factor
} IR_GP2Y0A41_t;

//   _____                 _   _
//  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
//  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
//  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/

IR_GP2Y0A41_t *IR_sens_new(uint8_t _pin, uint8_t v_supply, uint8_t bits){
    IR_GP2Y0A41_t *s = malloc(sizeof(IR_GP2Y0A41_t));
    if(!s){
        serial_e("Error allocating memory for IR sensor");
        return NULL;
    }

    memset(s,0,sizeof(IR_GP2Y0A41_t));

    s->pin = _pin;
    s->k   = (data_t) v_supply / bits;

    return s;
}

void IR_sens_free(IR_GP2Y0A41_t *s){
    assert(s);
    free(s);
}

data_t IR_read(IR_GP2Y0A41_t const *s){
    assert(s);

    return 13 / (s->k * analogRead(s->pin));
     
}

data_t IR_read_filtered(IR_GP2Y0A41_t const *s){
    assert(s);
    data_t m = 0;
    for (uint8_t i = 0; i < IR_AVERAGE_READ; i++){
        m += IR_read(s);
    }
    
    return m / IR_AVERAGE_READ;
}


#ifdef IR_GP2Y0A41_MAIN

#include "IR_GP2Y0A41.h"

IR_GP2Y0A41_t *ir_sens;

void setup() {
    Serial.begin(9600);

    ir_sens = IR_sens_new(A0, 5, 1024);
    if (!ir_sens) {
        Serial.println("Allocazione fallita!");
        while (1); 
    }

    Serial.println("Test IR_GP2Y0A41 Start");
}

void loop() {
    data_t raw_value = IR_read(ir_sens);
    data_t filtered_value = IR_read_filtered(ir_sens);

    Serial.print("Raw Value: ");
    Serial.print(raw_value);
    Serial.print(" | Filtered Value: ");
    Serial.println(filtered_value);

    delay(500);
}

#endif
