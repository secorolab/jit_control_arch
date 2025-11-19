// controllers.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


double evaluate_equality_constraint(double quantity, double reference);
double evaluate_less_than_constraint(double quantity, double threshold);
double evaluate_greater_than_constraint(double quantity, double threshold);
double evaluate_bilateral_constraint(double quantity, double lower, double upper);

void saturate(double *value, double min, double max);

double p_controller(double p_gain, double error);

double pi_controller(double p_gain, double i_gain, double error, double *integral_state, double dt);

double pid_controller(double p_gain, double i_gain, double d_gain,
                      double error, double *integral_state,
                      double error_sum_tol, double decay_rate,
                      double *prev_error, double dt);

#ifdef __cplusplus
}
#endif

