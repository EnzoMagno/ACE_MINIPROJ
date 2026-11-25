// Hardware drivers: encoders, motors, battery and IR line sensors
#ifndef HW_H
#define HW_H

#include <Arduino.h>
#include "config.h"

// Encoders -----------------------------------------------------------------
void encoders_start(void);           // call on core 1: the sampling interrupt runs there
int32_t encoder1_ticks(void);        // cumulative ticks (only the interrupt writes them)
int32_t encoder2_ticks(void);

// Motors -------------------------------------------------------------------
void motors_init(void);
void motors_set_volts(float v1, float v2);   // uses the measured battery voltage
float max_motor_volts(void);

// Battery ------------------------------------------------------------------
void battery_init(void);
void battery_update(void);           // reads the bytes received on BATTERY_RX_PIN
float battery_volts(void);
void battery_report(Print &out);

// IR line sensors ----------------------------------------------------------
struct ir_data_t {
  uint16_t raw[IR_COUNT];            // 0..1023, larger = darker (line)
};

void ir_init(void);
void ir_read(ir_data_t &d);          // ~0.2 ms (mux settling)
void ir_cal_reset(void);             // start a calibration: forget min/max
void ir_cal_update(const ir_data_t &d);
// Line position (m, 0 = centre) from the normalized readings.
// Returns false when no sensor sees the line.
bool ir_line_position(const ir_data_t &d, float &pos);
void ir_report(Print &out, const ir_data_t &d);

#endif // HW_H
