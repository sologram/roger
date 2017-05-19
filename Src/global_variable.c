/* exact-width signed integer types 
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __INT64 int64_t;
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __INT64 uint64_t;*/

#include "stdint.h"

//variable for uart communication
uint8_t rx_len;
uint8_t txdata[50];
uint8_t rxdata[50];
uint8_t flag_rx;
uint8_t flag_closepower;
uint8_t flag_throttle;
uint8_t heatstep;
uint8_t counter_syntony;

uint16_t ad_voltage;
uint16_t ad_current;
uint16_t ad_sink;
uint16_t ad_pan;
uint16_t poweroff_delay;

uint16_t pwm_pulse;
uint16_t target_power;







