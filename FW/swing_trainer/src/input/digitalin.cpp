#include "../../include/input/digitalin.h"

typedef struct digitalin_t {
    uint8_t pin;       ///< Pin number associated with the digital input.
    unsigned long t;         ///< Debounce time in milliseconds.
    bool reverse;      ///< Logic reversal flag.
} digitalin_t;

digitalin_t* digital_new(uint8_t _pin, uint8_t debounce_time, bool internal_pullup,bool reverse_logic){
    digitalin_t *i = malloc(sizeof(digitalin_t));
    if(!i){
        serial_e("Can't allocate memory for digitalin_t!");
        return NULL;
    }

    memset(i, 0, sizeof(digitalin_t));
    (internal_pullup) ? pinMode(_pin, INPUT_PULLUP) : pinMode(_pin, INPUT);
    
    i->pin = _pin;
    i->t = debounce_time;
    i->reverse = reverse_logic;

    return i;
}

void digital_free(digitalin_t *i){
    assert(i);
    free(i);
}

bool digital_read(digitalin_t *i){
    assert(i);
    return i->reverse ^ digitalRead(i->pin);
}

bool digital_new_status(digitalin_t *i, unsigned long t_stamp){
    return (t_stamp - i->t) >= DEBOUNCE_TIME ? (i->t = t_stamp, true) : false;
}


