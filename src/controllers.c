#include <math.h>
#include "jit_control_arch/controllers.h"


double evaluate_equality_constraint(double quantity, double reference) {
    return quantity - reference;
}

double evaluate_less_than_constraint(double quantity, double threshold) {
    return (quantity < threshold) ? 0.0 : threshold - quantity;
}

double evaluate_greater_than_constraint(double quantity, double threshold) {
    return (quantity > threshold) ? 0.0 : quantity - threshold;
}

double evaluate_bilateral_constraint(double quantity, double lower, double upper) {
    if (quantity < lower)
        return lower - quantity;
    else if (quantity > upper)
        return quantity - upper;
    else
        return 0.0;
}


void saturate(double *value, double min, double max) {
    if (*value < min) {
        *value = min;
    } else if (*value > max) {
        *value = max;
    }
}


double p_controller(double p_gain, double error)
{
    return p_gain * error;
}

double pi_controller(double p_gain, double i_gain, double error,
                     double *integral_state, double dt)
{
    *integral_state += error * dt;
    return p_gain * error + i_gain * (*integral_state);
}

double pid_controller(double p_gain, double i_gain, double d_gain,
                      double error, double *integral_state,
                      double error_sum_tol, double decay_rate,
                      double *prev_error, double dt)
{
    double derivative = (error - *prev_error) / dt;

    if (fabs(error) > 0.0) {
        // accumulate integral only if error is non-zero
        *integral_state += error * dt;

        // clamp integral state to prevent windup
        if (fabs(error) > 0.0) {
            *integral_state += error * dt;

            if (*integral_state > error_sum_tol) {
                *integral_state = error_sum_tol;
            } else if (*integral_state < -error_sum_tol) {
                *integral_state = -error_sum_tol;
            }
        }
    } else {
        // decay integral state towards zero when error is zero
        *integral_state = decay_rate * (*integral_state) + (1.0 - decay_rate) * 0.0;
    }

    *prev_error = error;
    return p_gain * error + i_gain * (*integral_state) + d_gain * derivative;
}

