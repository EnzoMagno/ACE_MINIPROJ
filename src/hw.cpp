#include "hw.h"
#include <SerialPIO.h>
#include <pico/time.h>

// ===========================================================================
// Encoders: both channels sampled at ENCODER_SAMPLE_HZ, x4 quadrature decoding.
// The counters are cumulative and written only by the interrupt, so readers on
// any core just read them (32-bit reads are atomic) and take differences.
// ===========================================================================
static volatile int32_t enc1_pos = 0, enc2_pos = 0;
static int enc1_state = 0, enc2_state = 0;
static const int8_t encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
static repeating_timer_t enc_timer;

#define pinIsHigh(pin, pins)  (((pins) >> (pin)) & 1)

static bool encoder_isr(repeating_timer_t *)
{
  uint32_t pins = sio_hw->gpio_in;   // one read for all pins

  int next = (pinIsHigh(ENC1_A, pins) << 1) | pinIsHigh(ENC1_B, pins);
  enc1_pos += encoder_table[(enc1_state << 2) | next];
  enc1_state = next;

  next = (pinIsHigh(ENC2_A, pins) << 1) | pinIsHigh(ENC2_B, pins);
  enc2_pos -= encoder_table[(enc2_state << 2) | next];   // motor 2 is mirrored
  enc2_state = next;
  return true;
}

void encoders_start(void)
{
  pinMode(ENC1_A, INPUT_PULLUP);  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);  pinMode(ENC2_B, INPUT_PULLUP);

  // A pool created here fires its interrupt on the calling core (core 1)
  alarm_pool_t *pool = alarm_pool_create_with_unused_hardware_alarm(4);
  alarm_pool_add_repeating_timer_us(pool, -(1000000 / ENCODER_SAMPLE_HZ), encoder_isr, NULL, &enc_timer);
}

int32_t encoder1_ticks(void) { return enc1_pos; }
int32_t encoder2_ticks(void) { return enc2_pos; }

// ===========================================================================
// Motors (H-bridge: both inputs high = brake)
// ===========================================================================
static float vbat = BATTERY_DEFAULT_V;

void motors_init(void)
{
  pinMode(MOTOR1A_PIN, OUTPUT);  pinMode(MOTOR1B_PIN, OUTPUT);
  pinMode(MOTOR2A_PIN, OUTPUT);  pinMode(MOTOR2B_PIN, OUTPUT);
  motors_set_volts(0, 0);
}

static void set_motor_pwm(int pwm, int pin_a, int pin_b)
{
  pwm = constrain(pwm, -PWM_LIMIT, PWM_LIMIT);
  if (pwm == 0)      { analogWrite(pin_a, 255);       analogWrite(pin_b, 255); }
  else if (pwm > 0)  { analogWrite(pin_a, 255 - pwm); analogWrite(pin_b, 255); }
  else               { analogWrite(pin_a, 255);       analogWrite(pin_b, 255 + pwm); }
}

static int volts_to_pwm(float volts)
{
  float vb = (vbat < BATTERY_OFFSET_V) ? BATTERY_OFFSET_V : vbat;  // never divide by an invalid value
  return constrain((int) lroundf(volts / vb * 255.0f), -PWM_LIMIT, PWM_LIMIT);
}

void motors_set_volts(float v1, float v2)
{
  set_motor_pwm(volts_to_pwm(v1), MOTOR1A_PIN, MOTOR1B_PIN);
  set_motor_pwm(volts_to_pwm(v2), MOTOR2A_PIN, MOTOR2B_PIN);
}

float max_motor_volts(void) { return PWM_LIMIT / 255.0f * vbat; }

// ===========================================================================
// Battery: one byte on BATTERY_RX_PIN, V = 4.8 + (b >> 1) * 0.05
// ===========================================================================
static SerialPIO SerialTiny(NOPIN, BATTERY_RX_PIN);
static uint8_t battery_raw = 0;
static bool battery_valid = false;
static unsigned long battery_last_ms = 0;

void battery_init(void) { SerialTiny.begin(115200); }

void battery_update(void)
{
  while (SerialTiny.available() > 0) {
    battery_raw = (uint8_t) SerialTiny.read();
    vbat = BATTERY_OFFSET_V + (battery_raw >> 1) * BATTERY_STEP_V;
    battery_valid = true;
    battery_last_ms = millis();
  }
}

float battery_volts(void) { return vbat; }

void battery_report(Print &out)
{
  if (!battery_valid) { out.println("# bat: no byte received yet (using the default)"); return; }
  out.printf("# bat: raw %u, %.2f V, last byte %lu ms ago%s\n", battery_raw, vbat,
                millis() - battery_last_ms, vbat < BATTERY_LOW_V ? "  LOW BATTERY" : "");
}

// ===========================================================================
// IR line sensors through the analog multiplexer
// ===========================================================================
static uint16_t ir_white[IR_COUNT], ir_black[IR_COUNT];   // calibration (per sensor)

void ir_init(void)
{
  pinMode(MUXA_PIN, OUTPUT);  pinMode(MUXB_PIN, OUTPUT);  pinMode(MUXC_PIN, OUTPUT);
  analogReadResolution(10);
  for (int i = 0; i < IR_COUNT; i++) { ir_white[i] = IR_WHITE_DEFAULT; ir_black[i] = IR_BLACK_DEFAULT; }
}

void ir_read(ir_data_t &d)
{
  for (int i = 0; i < IR_COUNT; i++) {
    int ch = IR_MUX_FIRST + i;
    digitalWrite(MUXA_PIN, ch & 1);
    digitalWrite(MUXB_PIN, (ch >> 1) & 1);
    digitalWrite(MUXC_PIN, (ch >> 2) & 1);
    delayMicroseconds(IR_SETTLE_US);
    d.raw[IR_COUNT - 1 - i] = 1023 - analogRead(IR_ADC_PIN);   // same order as MicroRato
  }
}

void ir_cal_reset(void)
{
  for (int i = 0; i < IR_COUNT; i++) { ir_white[i] = 1023; ir_black[i] = 0; }
}

void ir_cal_update(const ir_data_t &d)
{
  for (int i = 0; i < IR_COUNT; i++) {
    if (d.raw[i] < ir_white[i]) ir_white[i] = d.raw[i];
    if (d.raw[i] > ir_black[i]) ir_black[i] = d.raw[i];
  }
}

bool ir_line_position(const ir_data_t &d, float &pos)
{
  // Weighted centroid of the normalized readings: gives a continuous position
  // between sensors, which is what makes the steering smooth.
  float sum = 0, wsum = 0;
  for (int i = 0; i < IR_COUNT; i++) {
    float span = (float) ir_black[i] - ir_white[i];
    float n = (span > 10) ? ((float) d.raw[i] - ir_white[i]) / span : 0;
    n = constrain(n, 0.0f, 1.0f);
    sum  += n;
    wsum += n * (i - (IR_COUNT - 1) / 2.0f) * IR_SPACING_M;
  }
  if (sum < IR_LINE_MIN_SUM) return false;
  pos = wsum / sum;
  return true;
}

void ir_report(Print &out, const ir_data_t &d)
{
  out.print("# ir raw:");
  for (int i = 0; i < IR_COUNT; i++) out.printf(" %4u", d.raw[i]);
  out.print("  white:");
  for (int i = 0; i < IR_COUNT; i++) out.printf(" %4u", ir_white[i]);
  out.print("  black:");
  for (int i = 0; i < IR_COUNT; i++) out.printf(" %4u", ir_black[i]);
  float pos;
  if (ir_line_position(d, pos)) out.printf("  pos: %.1f mm\n", pos * 1000);
  else out.println("  pos: no line");
}
