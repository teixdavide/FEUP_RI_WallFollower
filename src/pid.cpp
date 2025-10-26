#include <pid/pid.hpp>

#include <fstream>

PID::PID(double p, double i, double d, double aw_limit)
    : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0), anti_windup_limit(aw_limit) {}


double PID::calculate_proportional(double error) {
    return kp * error;
}

double PID::calculate_integral(double error, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }
    integral += error * dt;
    // Anti-windup: Clamp the integral term
    if (integral > anti_windup_limit) {
        integral = anti_windup_limit;
    } else if (integral < -anti_windup_limit) {
        integral = -anti_windup_limit;
    }
    return ki * integral;
}

double PID::calculate_derivative(double error, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }
    double derivative = (error - prev_error) / dt;
    prev_error = error; // Update previous error
    return kd * derivative;
}

double PID::calculate_control(double setpoint, double measured_value, double dt, bool leader) {
    double error = setpoint - measured_value;

    std::ofstream logfile(leader ? "leader_pid_error.txt" : "follower_pid_error.txt", std::ios::app);
    if (logfile.is_open()) {
        logfile << error << std::endl;
    }

    double p_term = calculate_proportional(error);
    double i_term = calculate_integral(error, dt);
    double d_term = calculate_derivative(error, dt);

    // Anti-windup: Clamp the output if necessary
    double output = p_term + i_term + d_term;
    if (output > anti_windup_limit) {
        output = anti_windup_limit;
    } else if (output < -anti_windup_limit) {
        output = -anti_windup_limit;
    }
    return output;
}
