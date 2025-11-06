// controllers.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Simple proportional controller
double p_controller(double p_gain, double error);

// Proportional-Integral controller
double pi_controller(double p_gain, double i_gain, double error, double *integral_state, double dt);

// Proportional-Integral-Derivative controller
double pid_controller(double p_gain, double i_gain, double d_gain,
                      double error, double *integral_state,
                      double *prev_error, double dt);

#ifdef __cplusplus
}
#endif

