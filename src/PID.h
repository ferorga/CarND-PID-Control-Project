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

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool I_sat;
  double target;
  double min;
  double max;
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
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  /*
  * Calculate steering value
  */
  double GetOutput();

  /*
  * Set Kp gain
  */
  void SetKp(double Kp_);

  /*
  * Set Kd gain
  */
  void SetKd(double Kd_);


  /*
  * Set PID output target value (offset)
  */
  void SetTarget(double target_);

  /*
  * Set PID max limit
  */
  void SetMax(double max_);

  /*
  * Set PID min limit
  */
  void SetMin(double min_);
};

#endif /* PID_H */