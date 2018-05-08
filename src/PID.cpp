#include "PID.h"

//using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {

  // store previous error
  double prev_cte = p_error;

  // current cross-track-error. direct proportional component
  p_error  = cte;

  // integral error - summation of all errors
  i_error += cte;

  // error delta. differential component
  d_error  = cte - prev_cte;
}

double PID::TotalError() {
  return -(p_error * Kp + i_error * Ki + d_error * Kd);
}

