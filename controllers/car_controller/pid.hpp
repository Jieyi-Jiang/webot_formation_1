#pragma once
#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID();
    PID(double kp, double ki, double kd, double min, double max);
    ~PID();

    double update_inc(double error);
    double update_pos(double error);
    // double output();
    void reset();

private:
    // double _aim;
    double _kp;
    double _ki;
    double _kd;
    double _dt;
    double _max;
    double _min;
    double _e_k0;
    double _e_k1;
    double _e_k2;
    double _integral;
    double _output;
};

#endif // PID.hpp