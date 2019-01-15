#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;    

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    I_sat = false;
    t0 = std::chrono::high_resolution_clock::now();
    lpf.setCutOffFrequency(15); // 15 * 2 * pi Hz 
}

void PID::UpdateError(double cte) 
{
  // Differential time
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  t0 = std::chrono::high_resolution_clock::now();
  double dt = ms.count()/1000.0;

  // Differential error: Low-pass filter the differential of the cte
  d_error = lpf.update((cte - p_error)/dt, dt);

  // Integral error: Limit the integral error to avoid wind-up
  if (p_error > 0.0 && cte < 0.0)
    i_error = 0.0;
  else if (p_error < 0.0 && cte > 0.0)
    i_error = 0.0;
  else if (!I_sat)
    i_error += cte * dt;

  // Proportional error
  p_error = cte;  
}

double PID::GetOutput() 
{
    double out = 0.0;
    double u = 0.0;  

    // Proportional
    double P = Kp * p_error;

    // Differential
    double D = d_error * Kd;
    if (D > 0.1)
      D = 0.1;
    else if (D < -0.1)
      D = -0.1;

    // Integral
    double I = (i_error * Ki);
    if (I > 0.1)
    {
      I = 0.1;
      I_sat = true;
    }
    else if (I < -0.1)
    {
      I = -0.1;
      I_sat = true;
    }
    else
    {
      I_sat = false;
    }

    // PID Output
    out = target - (P + I + D);

    // Output Limit
    if (out > max)
    {   
      u = max;
    }
    else if (out < min)
    {
      u = min;
    }
    else
    {
      u = out;
    }

    return u;
}

void PID::SetKp(double Kp_)
{
  Kp = Kp_;
}

void PID::SetKd(double Kd_)
{
  Kd = Kd_;
}

void PID::SetTarget(double target_)
{
  target = target_;
}

void PID::SetMax(double max_)
{
  max = max_;
}

void PID::SetMin(double min_)
{
  min = min_;
}