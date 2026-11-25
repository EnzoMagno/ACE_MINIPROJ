#include "PID.h"
#include "config.h"

PID_t::PID_t()
{
  dt = CONTROL_PERIOD_MS * 1e-3f;
  Kp = SPEED_KP;  Ki = SPEED_KI;  Kd = SPEED_KD;  Kf = SPEED_KF;
  m_max = 5.8f;  m_min = -5.8f;
  reset();
}

void PID_t::reset(void)
{
  e = last_e = Se = m = 0;
}

float PID_t::calc(float ref, float measured)
{
  last_e = e;
  e = ref - measured;
  Se += e * dt;
  float de = (e - last_e) / dt;

  m = Kp * e + Ki * Se + Kd * de + Kf * ref;

  // Anti-windup: saturated and the error pushes further out -> stop integrating
  if ((m > m_max && e > 0) || (m < m_min && e < 0)) Se -= e * dt;

  if (m > m_max) m = m_max;
  else if (m < m_min) m = m_min;
  return m;
}
