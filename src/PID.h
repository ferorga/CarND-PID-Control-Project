#ifndef PID_H
#define PID_H

#include <chrono>
#include "LowPassFilter.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  bool I_sat;
  double target;
  double min;
  double max;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double Kt;
  std::chrono::high_resolution_clock::time_point t0;
  LowPassFilter lpf;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  /*
  * Calculate steering value
  */
  double GetOutput();

  void SetKp(double Kp_);

  void SetKd(double Kd_);

  void SetTarget(double target_);

  void SetMax(double max_);

  void SetMin(double min_);
};

#endif /* PID_H */