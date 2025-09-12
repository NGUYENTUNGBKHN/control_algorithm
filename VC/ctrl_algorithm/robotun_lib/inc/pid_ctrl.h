#pragma once

class pid_ctrl
{
public:
    pid_ctrl(double Kp_, double Ki_, double Kd_, double target_, double last_state_ = 0);
    ~pid_ctrl();

    double get_ctrl(double value, double dt);
    void set_target(double target_);
private:
    double Kp, Ki, Kd;
    double target;
    double last_state;
    double integral_eror;
protected:
};



