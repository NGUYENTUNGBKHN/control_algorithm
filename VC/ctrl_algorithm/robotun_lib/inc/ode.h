#pragma once

#include <vector>

class ODE
{
    typedef enum ODE_STYLE
    {
        ODE_STYLE_EULER,
        ODE_STYLE_RK4,
    } ODE_STYLE;
	typedef std::vector<double>(*derivative_t)(std::vector<double> state, int step, double t, double dt);

public:
    ODE(std::vector<double> state_,
        std::vector<double> times_,
		derivative_t derivative_,
		ODE_STYLE style_ = ODE_STYLE_RK4
        );
    ~ODE();

	std::vector<double> integrate_euler(std::vector<double> state, int step, double t, double dt, derivative_t derivative_func);
	std::vector<double> integrate_rk4(std::vector<double> state, int step, double t, double dt, derivative_t derivative_func);
	std::vector<std::vector<double>> solve();
private:
	std::vector<double> state;
	std::vector<double> times;
	derivative_t derivative;
	ODE_STYLE style;
protected:

};

