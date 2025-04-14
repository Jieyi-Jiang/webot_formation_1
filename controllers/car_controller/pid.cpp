#include "pid.hpp"
#include <cmath>
#include <iostream>

PID::PID()
{
    this->_kp = 0;
    this->_ki = 0;
    this->_kd = 0;
    this->_min = 0;
    this->_max = 0;
    this->_integral = 0;
    this->_e_k0 = 0;
    this->_e_k1 = 0;
    this->_e_k2 = 0;
}


PID::PID(double kp, double ki, double kd, double min, double max)
{
    this->_kp = kp;
    this->_ki = ki;
    this->_kd = kd;
    this->_min = min;
    this->_max = max;
    this->_integral = 0;
    this->_e_k0 = 0;
    this->_e_k1 = 0;
    this->_e_k2 = 0;
}

PID::~PID()
{
    ;
}

void PID::reset()
{
    this->_integral = 0;
    this->_e_k0 = 0;
    this->_e_k1 = 0;
    this->_e_k2 = 0;
}

double PID::update_pos(double error)
{
    _e_k0 = error;
    // if (_e_k0 < _max && _e_k0 > _min) _integral += _e_k0;
    if (_e_k0 < _max*_kp && _e_k0 > _min*_ki) _integral += _e_k0;
    if (std::isnan(_integral)) _integral = 0;
    double derivative = _e_k0 - _e_k1;
    double output = _kp * _e_k0 + _ki * _integral + _kd * derivative;
    _e_k1 = _e_k0;
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;
    if (error < _max*_kp*0.1 && error > _min*_kp*0.1)
    {
        this->reset();
        output = 0.0;
    }
    return output;
}

double PID::update_inc(double error)
{
    _e_k0 = error;
    
    double output = _kp * (_e_k0 - _e_k1) + _ki * _e_k0 + _kd * (_e_k0 - 2 * _e_k1 + _e_k2);
    _e_k2 = _e_k1;
    _e_k1 = _e_k0;
    if (output > _max)
        return _max;
    else if (output < _min)
        return _min;
    return output;
}
