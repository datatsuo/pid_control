#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool do_twiddle) {

  // initialize the coefficients for PID
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // initialize error for each term of PID
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // initializations for parameters/variables used for twiddle
  // (for details, see PID.h)
  twiddle = do_twiddle;
  coeff_tuning = false; // this is used for skipping the first n_update steps when tuning coefficients
  tolerance = 0.01;
  case_id = 0;
  tune_id = 0;
  n_step = 0;
  n_update = 50;
  sum_error2 = 0.0;
  // set K[] to Kp, Ki, Kd
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
  // initialize dK
  dK[0] = 0.1 * Kp;
  dK[1] = 0.1 * Ki;
  dK[2] = 0.1 * Kd;

}


void PID::UpdateError(double cte) {
  // update errors for PID individually
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  // combine all the terms for PID to compute total error
  double total_error = Kp * p_error + Ki * i_error + Kd * d_error;

  return total_error;
}

void PID::TwiddleUpdate(double ave_error2){

    switch(case_id){

      case 0: { // after dK[tune_id] is added to K[tune_id]

        if(ave_error2 < best_error2){
            best_error2 = ave_error2;
            dK[tune_id] *= 1.01;
            // switch the coefficient to tune (Kp -> Ki -> Kd -> Kp -> Ki -> ...)
            tune_id = (tune_id + 1) % 3;
            K[tune_id] += dK[tune_id];
        }
        else{
            // next consider subtraction of dK[tune_id] from K[tune_id] instead
            K[tune_id] -= 2.0 * dK[tune_id];
            // go to case 1
            case_id = 1;
        }

        break;
      }

      case 1:{ //after dK[tune_id] is subtracted from K[tune_id]

        if(ave_error2 < best_error2){
            best_error2 = ave_error2;
            dK[tune_id] *= 1.01;
        }
        else{
            K[tune_id] += dK[tune_id];
            dK[tune_id] *= 0.99;
        }

        // switch the coefficient to tune and add dK[tune_id] to K[tune_id]
        tune_id = (tune_id + 1) % 3;
        K[tune_id] += dK[tune_id];
        // back to case 0
        case_id = 0;

        break;
      }

    }

}

void PID::UpdateErrorTwiddle(double cte) {

  // update errors for PID
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  // update n_step and sum_error2
  n_step += 1;
  sum_error2 += cte * cte;

  if(n_step == n_update){

    if(coeff_tuning == false){ //after the first n_update steps, start tuning coefficients
      K[0] += dK[0];
      best_error2 = sum_error2 / n_step;
      coeff_tuning = true;
    }
    else{ //every n_update steps, update the PID coeffs, if Kp+Ki+Kd is bigger than tolerance
      if((dK[0] + dK[1] + dK[2]) > tolerance){
        TwiddleUpdate(sum_error2/n_step);
      }
    }

    // substitute the updated PID coefficients to Kp, Ki, Kd.
    Kp = K[0];
    Ki = K[1];
    Kd = K[2];

    // reset sum_error2 and n_step used for twiddle
    sum_error2 = 0.0;
    n_step = 0;

    // display Kp, Ki, Kd to keep track of the PID coeffficients update
    cout << "PID coefficients (Kp, Ki, Kd): " << Kp << ", " << Ki << ", " << Kd << endl;
    cout << " " << endl;

  }

}
