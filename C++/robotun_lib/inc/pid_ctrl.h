#pragma one

class pid_ctrl
{
public:
    pid_ctrl(float Kp_, float Ki_, float Kd_, float target_, float init_state_ = 0);
    ~pid_ctrl();

    float get_ctrl(float value, float dt);
    void set_target(float target);
private:
    float Kp, Ki, Kd;
    float target;
    float init_state;
protected:
};



