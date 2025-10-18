#pragma once

class PID {
    private:
        double kp; // Proportional gain
        double ki; // Integral gain
        double kd; // Derivative gain
        double prev_error; // Previous error value
        double integral; // Integral of the error
        double anti_windup_limit; // Anti-windup limit

    public:
        // Constructor to initialize PID gains and anti-windup limit
        PID(double p, double i, double d, double aw_limit);

        double calculate_proportional(double error);

        double calculate_integral(double error, double dt);

        double calculate_derivative(double error, double dt);

        double calculate_control(double setpoint, double measured_value, double dt);

};
