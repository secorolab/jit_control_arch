// controllers.c
#include "jit_control_arch/controllers.h"

// P controller: u = Kp * e
double p_controller(double p_gain, double error)
{
    return p_gain * error;
}

// PI controller: u = Kp*e + Ki*∫e dt
double pi_controller(double p_gain, double i_gain, double error,
                     double *integral_state, double dt)
{
    *integral_state += error * dt;
    return p_gain * error + i_gain * (*integral_state);
}

// PID controller: u = Kp*e + Ki*∫e dt + Kd*(de/dt)
double pid_controller(double p_gain, double i_gain, double d_gain,
                      double error, double *integral_state,
                      double *prev_error, double dt)
{
    double derivative = (error - *prev_error) / dt;
    *integral_state += error * dt;
    *prev_error = error;
    return p_gain * error + i_gain * (*integral_state) + d_gain * derivative;
}

