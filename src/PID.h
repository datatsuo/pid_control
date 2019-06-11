#ifndef PID_H
#define PID_H

#include <iostream>

class PID {
public:
  /*
  * error for each term in PID
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * PID coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  the followings are used for the parameter search with twiddle
  */
  // number of time steps after the last parameter update
  int n_step;
  // the number of time steps used for computing errors in twiddle
  int n_update;
  // case index for twiddle (case_id = 0 or 1). see TwiddleUpdate() for the detail
  int case_id;
  // index labeling which coefficient is under tuning with twiddle
  // (0: Kp, 1: Ki, 2: Kd)
  int tune_id;
  // if the parameter tuning is going on now or not
  bool coeff_tuning;
  // whether to carry out the tuning of PID coefficients with twiddle or not
  bool twiddle;
  // Kp, Ki, and Kd are stored
  double K[3];
  // shift of Kp, Ki, Kd under twiddle
  double dK[3];
  // tolerance for the update with twiddle
  double tolerance;
  // sum of cross track error squared during single coefficient update process
  double sum_error2;
  // the best (average) cross track error squared
  double best_error2;

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
  void Init(double Kp_, double Ki_, double Kd_, bool do_twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  *  Parameter tuning with with Twiddle
  */
  void TwiddleUpdate(double ave_error2);

  /*
  * Update the PID error variables given cross track error.
  * The coefficients for PID are also updated with twiddle
  */
  void UpdateErrorTwiddle(double cte);

};

#endif /* PID_H */
