// PID with feedforward and anti-windup (based on Paulo Costa's PID_t, LAB1)
#ifndef PID_H
#define PID_H

class PID_t
{
  public:
    float dt;
    float Kp, Ki, Kd, Kf;
    float e, last_e, Se;     // error, previous error, integral of the error
    float m, m_max, m_min;   // output and its limits

    PID_t();
    void reset(void);
    float calc(float ref, float measured);
};

#endif // PID_H
