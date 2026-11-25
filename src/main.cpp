// ===========================================================================
// ACE_MINIPROJ - line follower with encoder speed control on FreeRTOS
//
//  core 1  encoder interrupt (ENCODER_SAMPLE_HZ)
//          sensor_task   (SENSOR_PERIOD_MS)   reads the IR sensors
//          control_task  (CONTROL_PERIOD_MS)  line follower -> v, w -> wheel rpm
//                                             -> speed PID + feedforward -> motors
//  core 0  setup()/loop()                     commands (USB and WiFi), buttons, telemetry
//
// Only loop() prints (on USB and on the WiFi client); the tasks on core 1 leave
// messages in g_event.
// All tunable values are in include/config.h.
// ===========================================================================
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <WiFi.h>

#include "config.h"
#include "hw.h"
#include "PID.h"
#include "commands.h"

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------
typedef enum { st_idle, st_follow, st_ir_cal, st_fwd } run_state_t;
static const char *state_name[] = {"idle", "follow", "ir_cal", "fwd"};

static volatile run_state_t g_state = st_idle;   // written by loop() and control_task

typedef enum { ev_none, ev_line_lost, ev_gap_lost, ev_cal_done, ev_fwd_done } event_t;
static volatile event_t g_event = ev_none;       // control_task -> loop() messages

static ir_data_t g_ir;                           // latest IR reading (sensor_task)

// Tunable at run time (cmd). Each is a 32-bit float: written on core 0, read on core 1.
static PID_t pid1, pid2;                         // right and left wheel speed (rpm -> V)
static volatile float line_kp = LINE_KP, line_ki = LINE_KI, line_kd = LINE_KD;
static volatile float line_sign = LINE_SIGN, v_nominal = V_NOMINAL, slowdown = SLOWDOWN;
static volatile float gap_center = GAP_CENTER_M, gap_max = GAP_MAX_M;
static volatile float fwd_kh = FWD_KH, fwd_ky = FWD_KY, fwd_accel = FWD_ACCEL;
static volatile float wheel_scale = WHEEL_RIGHT_SCALE;
static volatile float fwd_heading, fwd_y, fwd_dist;   // result of the last 'fwd' (odometry)
static volatile uint32_t fwd_ms = 0;             // 'fwd': duration; 'walk': timeout
static volatile float walk_m = 0;                // 'walk': distance (0 = 'fwd', time mode)

struct telemetry_t {
  uint32_t t_ms;
  float pos, w, ref1, rpm1, ref2, rpm2, u1, u2;
};
static telemetry_t g_tel;
static bool telemetry_on = true;

// ---------------------------------------------------------------------------
// Tasks (core 1)
// ---------------------------------------------------------------------------
static void sensor_task(void *)
{
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    ir_data_t d;
    ir_read(d);
    taskENTER_CRITICAL();
    g_ir = d;
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

static float ticks_to_rpm(int32_t ticks)
{
  return ticks * 60.0f / (TICKS_PER_REV * CONTROL_PERIOD_MS * 1e-3f);
}

// Wheel 1 (right) radius is WHEEL_RADIUS_M * wheel_scale: corrects different wheel sizes
static float wheel_speed_to_rpm(float v_mps, float radius)
{
  return v_mps / radius * 60.0f / TWO_PI;
}

static float rpm_to_wheel_speed(float rpm, float radius)
{
  return rpm * TWO_PI / 60.0f * radius;
}

static void control_task(void *)
{
  const float dt = CONTROL_PERIOD_MS * 1e-3f;
  encoders_start();                   // the encoder interrupt lives on this core

  int32_t last1 = encoder1_ticks(), last2 = encoder2_ticks();
  run_state_t prev_state = st_idle;
  float last_pos = 0, pos_int = 0, last_w = 0, lost_dist = 0, heading = 0, y = 0, dist = 0, v_ramp = 0;
  uint32_t lost_ms = 0, state_ms = 0, stop_ms = 0;
  bool gap = false;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    battery_update();

    int32_t n1 = encoder1_ticks(), n2 = encoder2_ticks();
    float rpm1 = ticks_to_rpm(n1 - last1), rpm2 = ticks_to_rpm(n2 - last2);
    last1 = n1;  last2 = n2;
    float r1 = WHEEL_RADIUS_M * wheel_scale, r2 = WHEEL_RADIUS_M;
    float v1e = rpm_to_wheel_speed(rpm1, r1), v2e = rpm_to_wheel_speed(rpm2, r2);

    ir_data_t ir;
    taskENTER_CRITICAL();
    ir = g_ir;
    taskEXIT_CRITICAL();

    run_state_t state = g_state;
    if (state != prev_state) {        // entering a state: start from a clean slate
      pid1.reset();  pid2.reset();
      pos_int = 0;  last_w = 0;  lost_ms = 0;  state_ms = 0;  heading = y = dist = v_ramp = 0;  stop_ms = 0;
      float p;
      last_pos = ir_line_position(ir, p) ? p : 0;
      if (state == st_ir_cal) ir_cal_reset();
      prev_state = state;
    }
    state_ms += CONTROL_PERIOD_MS;

    float v = 0, w = 0, pos = 0;

    if (state == st_follow) {
      if (ir_line_position(ir, pos)) {
        if (lost_ms > 0) last_pos = pos;   // line found again: no derivative kick
        lost_ms = 0;
        pos_int = constrain(pos_int + pos * dt, -0.05f, 0.05f);   // limits the integral (anti-windup)
        float dpos = (pos - last_pos) / dt;
        last_pos = pos;
        w = line_sign * (line_kp * pos + line_ki * pos_int + line_kd * dpos);
        w = constrain(w, -LINE_W_MAX, LINE_W_MAX);
        last_w = w;
      } else {
        if (lost_ms == 0) {             // just lost: gap or curve?
          gap = fabsf(last_pos) < gap_center;
          lost_dist = 0;
        }
        lost_ms += CONTROL_PERIOD_MS;
        lost_dist += fabsf(v1e + v2e) / 2 * dt;

        if (gap) {
          // Gap: the line was centred, so it continues ahead. Drive straight.
          w = 0;
          if (lost_dist >= gap_max) { g_state = st_idle; g_event = ev_gap_lost; }
        } else {
          // The line left the sensors at one side (tight curve): keep turning to that side
          w = last_w;
          if (lost_ms >= LINE_LOST_MS) { g_state = st_idle; g_event = ev_line_lost; }
        }
      }
      v = v_nominal / (1 + slowdown * fabsf(w));   // slows down in tight curves

    } else if (state == st_fwd) {
      // Straight line, odometry from the encoders: heading and lateral offset y
      heading += (v1e - v2e) / WHEEL_DIST_M * dt;
      y       += (v1e + v2e) / 2 * sinf(heading) * dt;
      dist    += (v1e + v2e) / 2 * dt;

      // Speed ramp: accelerate, cruise, and brake so it stops at the end
      float v_max_stop;
      if (walk_m > 0) {                 // 'walk': stop at the distance (v^2 = 2*a*d)
        float d_left = walk_m - dist;
        v_max_stop = sqrtf(2 * fwd_accel * fmaxf(d_left, 0));
        if (d_left > WALK_TOL_M) v_max_stop = fmaxf(v_max_stop, WALK_V_MIN);  // creep to the mark
        else if (stop_ms == 0) stop_ms = state_ms;
        if (state_ms >= fwd_ms && stop_ms == 0) stop_ms = state_ms;           // timeout
      } else {                          // 'fwd': stop at the time
        v_max_stop = fwd_accel * fmaxf((fwd_ms - (float) state_ms) * 1e-3f, 0);
        if (state_ms >= fwd_ms && stop_ms == 0) stop_ms = state_ms;
      }
      v_ramp = fminf(fminf(v_ramp + fwd_accel * dt, v_nominal), v_max_stop);
      if (stop_ms) v_ramp = 0;
      v = v_ramp;
      w = constrain(-fwd_kh * heading - fwd_ky * y, -LINE_W_MAX, LINE_W_MAX);
      pos = y;                          // telemetry column pos_mm = lateral offset
      if (stop_ms && state_ms >= stop_ms + 200) {   // 200 ms to settle at zero speed
        fwd_heading = heading;  fwd_y = y;  fwd_dist = dist;
        g_state = st_idle;  g_event = ev_fwd_done;
      }

    } else if (state == st_ir_cal) {
      ir_cal_update(ir);
      w = (state_ms < IR_CAL_MS / 2) ? IR_CAL_W : -IR_CAL_W;   // sweep both ways over the line
      if (state_ms >= IR_CAL_MS) { g_state = st_idle; g_event = ev_cal_done; }
    }

    if (g_state == st_idle) {
      motors_set_volts(0, 0);
      continue;
    }

    // Robot (v, w) -> wheel references. Motor 1 = right wheel.
    float ref1 = wheel_speed_to_rpm(v + w * WHEEL_DIST_M / 2, r1);
    float ref2 = wheel_speed_to_rpm(v - w * WHEEL_DIST_M / 2, r2);
    float biggest = fmaxf(fabsf(ref1), fabsf(ref2));
    if (biggest > MAX_WHEEL_RPM) {    // scale both: keeps the curve radius
      ref1 *= MAX_WHEEL_RPM / biggest;
      ref2 *= MAX_WHEEL_RPM / biggest;
    }

    pid1.m_max = pid2.m_max =  max_motor_volts();
    pid1.m_min = pid2.m_min = -max_motor_volts();
    float u1 = pid1.calc(ref1, rpm1);
    float u2 = pid2.calc(ref2, rpm2);
    motors_set_volts(u1, u2);

    taskENTER_CRITICAL();
    g_tel = {millis(), pos, w, ref1, rpm1, ref2, rpm2, u1, u2};
    taskEXIT_CRITICAL();
  }
}

// ---------------------------------------------------------------------------
// Output to USB and to the WiFi client at the same time (core 0 only)
// ---------------------------------------------------------------------------
static WiFiServer tcp_server(WIFI_TCP_PORT);
static WiFiClient tcp_client;

class tee_t : public Print {
  public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t n) override {
      Serial.write(buf, n);
      if (tcp_client && tcp_client.connected()) tcp_client.write(buf, n);
      return n;
    }
};
static tee_t out;

static void wifi_start(void)
{
#if WIFI_ENABLED
  WiFi.setHostname(WIFI_HOSTNAME);
#if WIFI_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  out.printf("# WiFi AP \"%s\" up, connect to %s port %d\n", WIFI_SSID,
             WiFi.softAPIP().toString().c_str(), WIFI_TCP_PORT);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_MS) delay(100);
  if (WiFi.status() == WL_CONNECTED)
    out.printf("# WiFi joined \"%s\", connect to %s port %d\n", WIFI_SSID,
               WiFi.localIP().toString().c_str(), WIFI_TCP_PORT);
  else
    out.printf("# WiFi: could not join \"%s\" (USB still works)\n", WIFI_SSID);
#endif
  tcp_server.begin();
#endif
}

// One client at a time; a new connection replaces the old one
static void wifi_update(commands_t &cmds)
{
#if WIFI_ENABLED
  WiFiClient c = tcp_server.accept();
  if (c) {
    if (tcp_client) tcp_client.stop();
    tcp_client = c;
    tcp_client.setNoDelay(true);
    out.println("# WiFi client connected. Send 'h' for the command list");
  }
  while (tcp_client && tcp_client.available()) {
    int b = tcp_client.read();
    if (b >= 0 && b < 0x80) cmds.process_char((char) b);   // drops telnet negotiation bytes
  }
#endif
}

// ---------------------------------------------------------------------------
// Commands (core 0) - the same for USB and WiFi
// ---------------------------------------------------------------------------
static commands_t serial_commands, wifi_commands;

static void print_params(void)
{
  out.printf("# state=%s  v=%.3f m/s  slow=%.3f  sign=%.0f\n", state_name[g_state],
             v_nominal, slowdown, line_sign);
  out.printf("# speed PID: kp=%.4f ki=%.4f kd=%.4f kf=%.4f\n", pid1.Kp, pid1.Ki, pid1.Kd, pid1.Kf);
  out.printf("# line: lkp=%.2f lki=%.2f lkd=%.3f   gap: gapc=%.4f m gapd=%.3f m\n",
             line_kp, line_ki, line_kd, gap_center, gap_max);
  out.printf("# fwd: fkh=%.2f fky=%.1f acc=%.2f   wr=%.4f\n", fwd_kh, fwd_ky, fwd_accel, wheel_scale);
}

static void print_help(void)
{
  out.println("# go st fwd walk | irc ir bat | p h tel 0/1 | v slow sign | kp ki kd kf | lkp lki lkd | gapc gapd | fkh fky acc wr");
}

static void process_command(frame_data_t f)
{
  out.println();                      // the command echo has no line end
  float x = f.value;

  if      (f.command_is("go"))  { g_state = st_follow; out.println("# t_ms,pos_mm,w,ref1,rpm1,ref2,rpm2,u1,u2"); }
  else if (f.command_is("st"))  g_state = st_idle;
  else if (f.command_is("fwd")) {           // 'fwd s': straight for s seconds at speed v
    if (g_state != st_idle || x <= 0) { out.println("# fwd: stop first ('st') and give the seconds"); return; }
    fwd_ms = (uint32_t) (x * 1000);
    walk_m = 0;
    g_state = st_fwd;
    out.printf("# fwd %.1f s at %.3f m/s\n", x, v_nominal);
    out.println("# t_ms,pos_mm,w,ref1,rpm1,ref2,rpm2,u1,u2");
    return;
  }
  else if (f.command_is("walk")) {          // 'walk m': straight for m metres at speed v
    if (g_state != st_idle || x <= 0 || v_nominal <= 0) { out.println("# walk: stop first ('st') and give the metres"); return; }
    walk_m = x;
    // timeout: twice the expected time (ramps included) + 2 s
    fwd_ms = (uint32_t) (2000 * (x / v_nominal + v_nominal / fwd_accel) + 2000);
    g_state = st_fwd;
    out.printf("# walk %.3f m at %.3f m/s\n", x, v_nominal);
    out.println("# t_ms,pos_mm,w,ref1,rpm1,ref2,rpm2,u1,u2");
    return;
  }
  else if (f.command_is("fkh")) fwd_kh = x;
  else if (f.command_is("fky")) fwd_ky = x;
  else if (f.command_is("acc")) fwd_accel = x;
  else if (f.command_is("wr"))  wheel_scale = x;
  else if (f.command_is("irc")) { if (g_state == st_idle) g_state = st_ir_cal; }
  else if (f.command_is("ir"))  { ir_data_t d; taskENTER_CRITICAL(); d = g_ir; taskEXIT_CRITICAL(); ir_report(out, d); return; }
  else if (f.command_is("bat")) { battery_report(out); return; }
  else if (f.command_is("h"))   { print_help(); return; }
  else if (f.command_is("tel")) telemetry_on = (x != 0);
  else if (f.command_is("v"))   v_nominal = x;
  else if (f.command_is("slow")) slowdown = x;
  else if (f.command_is("sign")) line_sign = (x < 0) ? -1 : 1;
  else if (f.command_is("kp"))  pid1.Kp = pid2.Kp = x;
  else if (f.command_is("ki"))  pid1.Ki = pid2.Ki = x;
  else if (f.command_is("kd"))  pid1.Kd = pid2.Kd = x;
  else if (f.command_is("kf"))  pid1.Kf = pid2.Kf = x;
  else if (f.command_is("lkp")) line_kp = x;
  else if (f.command_is("lki")) line_ki = x;
  else if (f.command_is("lkd")) line_kd = x;
  else if (f.command_is("gapc")) gap_center = x;
  else if (f.command_is("gapd")) gap_max = x;
  else if (!f.command_is("p"))  { out.println("# unknown command, send 'h'"); return; }

  print_params();
}

// Buttons: START toggles follow/idle, STOP always stops (falling edge, debounced)
static void buttons_update(void)
{
  static bool start_prev = false, stop_prev = false;
  static uint32_t last_edge = 0;
  bool start = digitalRead(START_BUTTON) == LOW;
  bool stop  = digitalRead(STOP_BUTTON) == LOW;

  if (millis() - last_edge > 50) {
    if (stop && !stop_prev) { g_state = st_idle; last_edge = millis(); }
    else if (start && !start_prev) {
      g_state = (g_state == st_idle) ? st_follow : st_idle;
      last_edge = millis();
    }
  }
  start_prev = start;  stop_prev = stop;
}

// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  motors_init();
  ir_init();
  battery_init();
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  serial_commands.init(process_command);
  wifi_commands.init(process_command);

  xTaskCreateAffinitySet(control_task, "control", 4096, NULL, CONTROL_PRIORITY, 1 << CONTROL_CORE, NULL);
  xTaskCreateAffinitySet(sensor_task,  "sensor",  2048, NULL, SENSOR_PRIORITY,  1 << SENSOR_CORE,  NULL);

  wifi_start();
  out.println("# ACE_MINIPROJ ready");
  print_help();
}

void loop()
{
  while (Serial.available()) serial_commands.process_char(Serial.read());
  wifi_update(wifi_commands);
  buttons_update();

  event_t ev = g_event;
  if (ev != ev_none) {
    g_event = ev_none;
    if (ev == ev_line_lost)     out.println("# line lost at the side: stopped");
    else if (ev == ev_gap_lost) out.println("# no line after the gap: stopped");
    else if (ev == ev_fwd_done)
      out.printf("# %s done: encoders say %.3f m, heading %.2f deg, lateral %.1f mm\n",
                 walk_m > 0 ? "walk" : "fwd", fwd_dist, fwd_heading * 180 / PI, fwd_y * 1000);
    else                        out.println("# IR calibration done (send 'ir' to check)");
  }

  static uint32_t last_tel = 0;
  if (telemetry_on && (g_state == st_follow || g_state == st_fwd) && millis() - last_tel >= TELEMETRY_PERIOD_MS) {
    last_tel = millis();
    telemetry_t t;
    taskENTER_CRITICAL();
    t = g_tel;
    taskEXIT_CRITICAL();
    out.printf("%lu,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f\n", t.t_ms, t.pos * 1000, t.w,
               t.ref1, t.rpm1, t.ref2, t.rpm2, t.u1, t.u2);
  }

  delay(5);   // frees core 0 for the other FreeRTOS tasks
}
