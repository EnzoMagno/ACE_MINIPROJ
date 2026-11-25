// ===========================================================================
// config.h - every parameter that can be tuned in ACE_MINIPROJ
//
// Values marked (cmd) can also be changed at run time over the serial port
// (see README.md). The values here are the defaults used after a reset.
// ===========================================================================
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Pins (Pico4Drive board, same as LAB1)
// ---------------------------------------------------------------------------
#define MOTOR1A_PIN   16      // motor 1 = RIGHT wheel
#define MOTOR1B_PIN   17
#define MOTOR2A_PIN   14      // motor 2 = LEFT wheel
#define MOTOR2B_PIN   15

#define ENC1_A        2
#define ENC1_B        3
#define ENC2_A        1
#define ENC2_B        0

#define MUXA_PIN      18      // IR sensor multiplexer address bits
#define MUXB_PIN      19
#define MUXC_PIN      20
#define IR_ADC_PIN    28      // multiplexer output (ADC2)
#define IR_MUX_FIRST  3       // mux channel of the first sensor (sensors on channels 3..7)

#define BATTERY_RX_PIN 21     // byte with the battery voltage, sent by the board

#define START_BUTTON  7       // starts / stops the run (active low)
#define STOP_BUTTON   27      // always stops (active low)

// ---------------------------------------------------------------------------
// FreeRTOS tasks: period (ms), priority and core
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_MS    20   // speed PIDs + line follower
#define SENSOR_PERIOD_MS     5    // IR sensors
#define TELEMETRY_PERIOD_MS  100  // CSV line on the serial port while running

#define CONTROL_PRIORITY     4
#define SENSOR_PRIORITY      3
#define CONTROL_CORE         1    // core 1: control + encoder interrupt
#define SENSOR_CORE          1
// setup()/loop() (serial commands, buttons, telemetry) run on core 0

#define ENCODER_SAMPLE_HZ    40000  // encoder sampling interrupt (core 1)

// ---------------------------------------------------------------------------
// Robot geometry and encoders
// ---------------------------------------------------------------------------
#define WHEEL_RADIUS_M       (0.070f / 2)   // wheel diameter 7 cm
#define WHEEL_DIST_M         0.105f
#define WHEEL_RIGHT_SCALE    1.0f     // right wheel radius / left wheel radius (cmd 'wr'), see README
#define ENCODER_TICKS_PER_MOTOR_REV 32.0f   // edges of A and B (x4) per motor turn
#define GEAR_RATIO           120.0f
#define TICKS_PER_REV        (ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO)  // per wheel turn

// ---------------------------------------------------------------------------
// Motors and battery
// ---------------------------------------------------------------------------
#define PWM_LIMIT            200      // largest PWM (out of 255)
#define BATTERY_DEFAULT_V    7.4f     // used until the first byte arrives
#define BATTERY_OFFSET_V     4.8f     // battery byte: V = 4.8 + (b >> 1) * 0.05
#define BATTERY_STEP_V       0.05f
#define BATTERY_LOW_V        6.4f     // warning threshold (2 x 18650)

// ---------------------------------------------------------------------------
// Wheel speed PID (input rpm, output Volts) - one per wheel   (cmd)
//   u = Kp*e + Ki*integral(e) + Kd*de/dt + Kf*rpm_ref
// ---------------------------------------------------------------------------
#define SPEED_KP             0.05f
#define SPEED_KI             0.5f
#define SPEED_KD             0.0f
#define SPEED_KF             0.038f   // feedforward: ~5 V for 130 rpm (curva_motores)
#define MAX_WHEEL_RPM        140.0f   // wheel references are limited to this

// ---------------------------------------------------------------------------
// IR line sensors
// ---------------------------------------------------------------------------
#define IR_COUNT             5
#define IR_SETTLE_US         40       // mux settling time before each ADC read
#define IR_WHITE_DEFAULT     200      // reading on the floor (0..1023) before calibration
#define IR_BLACK_DEFAULT     800      // reading on the line before calibration
#define IR_SPACING_M         0.0125f  // distance between neighbour sensors   <-- MEASURE
#define IR_LINE_MIN_SUM      0.4f     // sum of normalized readings below this = line lost
#define IR_CAL_MS            3000     // 'irc': time spinning in place to calibrate
#define IR_CAL_W             3.0f     // 'irc': spin speed (rad/s)

// ---------------------------------------------------------------------------
// Line follower (input: line position in m, output: robot angular speed rad/s)   (cmd)
//   w = sign * (Kp*pos + Ki*integral(pos) + Kd*dpos/dt)
//   v = V_NOMINAL / (1 + SLOWDOWN * |w|)       (slows down in tight curves)
// ---------------------------------------------------------------------------
#define LINE_SIGN            -1.0f    // line on the left gives pos < 0, and w > 0 turns left
#define LINE_KP              120.0f   // rad/s per m  (1 cm off -> 1.2 rad/s)
#define LINE_KI              200.0f   // removes the offset in circular curves (0 = pure PD)
#define LINE_KD              4.0f
#define LINE_W_MAX           8.0f     // largest angular speed (rad/s)
#define V_NOMINAL            0.25f    // forward speed on a straight line (m/s)
#define SLOWDOWN             0.15f    // speed reduction per rad/s of turn
#define LINE_LOST_MS         400      // line left the sensors on one side: keep the last turn this long, then stop

// ---------------------------------------------------------------------------
// 'fwd s': drive straight for s seconds, without the line   (cmd: v, fkh, fky, acc)
// Odometry from the encoders gives the heading error and the lateral offset y
// from the starting line:   w = -FWD_KH * heading - FWD_KY * y
// so the robot comes back to the starting line, not only parallel to it.
// The speed ramps up/down at FWD_ACCEL to avoid wheel slip at the start/stop.
// ---------------------------------------------------------------------------
#define FWD_KH               5.0f     // rad/s per rad of heading error
#define FWD_KY               25.0f    // rad/s per m of lateral offset (critical damping at 0.25 m/s)
#define FWD_ACCEL            0.5f     // m/s^2
#define WALK_V_MIN           0.03f    // 'walk m': slowest speed while approaching the mark (m/s)
#define WALK_TOL_M           0.004f   // 'walk m': cut the motors this far before the mark (the robot still coasts a few mm)

// ---------------------------------------------------------------------------
// Gaps in the line   (cmd)
// When the line disappears while it was near the centre of the sensors
// (|pos| < GAP_CENTER_M) it is a gap: the robot drives straight (w = 0) for at
// most GAP_MAX_M (measured with the encoders) looking for the line again.
// When it disappears at the side, it is a curve: see LINE_LOST_MS.
// ---------------------------------------------------------------------------
#define GAP_CENTER_M         0.008f   // line lost within +-8 mm of the centre = gap
#define GAP_MAX_M            0.15f    // longest gap crossed (m); then the robot stops

// ---------------------------------------------------------------------------
// WiFi (Pico W): commands and telemetry over TCP, same as the USB serial port
//   AP mode : the robot creates the network WIFI_SSID; connect to it and open
//             a raw TCP connection to 192.168.42.1, port WIFI_TCP_PORT
//   STA mode: the robot joins the network WIFI_SSID; its IP is printed on USB
// ---------------------------------------------------------------------------
#define WIFI_ENABLED         1
#define WIFI_AP_MODE         0        // 1 = robot is the access point, 0 = joins a network
#define WIFI_SSID            "iPhone de Enzo"
#define WIFI_PASS            "enzo310304"   // at least 8 characters
#define WIFI_HOSTNAME        "ace-robot"
#define WIFI_TCP_PORT        23
#define WIFI_CONNECT_MS      10000    // STA mode: give up after this long (USB keeps working)

#endif // CONFIG_H
