/******************************************************************************/
/*! @addtogroup Group2
    @file       free_fallcpp.cpp
    @brief      
    @date       2025/09/16
    @author     Development Dept at Tokyo (nguyen-thanh-tung@jcm-hq.co.jp)
    @par        Revision
    $Id$
    @par        Copyright (C)
    Japan CashMachine Co, Limited. All rights reserved.
******************************************************************************/

/*******************************************************************************
**                                INCLUDES
*******************************************************************************/
#include <cmath>
#include <matplot/matplot.h>
#include <model.h>
#include <ctrl.h>
#include <ode.h>
/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
robotun::ctrl::pid_ctrl th_pid = robotun::ctrl::pid_ctrl(40.0, 0.0, 20.0, 0.0);
robotun::ctrl::pid_ctrl velocity_pid = robotun::ctrl::pid_ctrl(0.0085, 0.001, 0.0, 0.0);
robotun::ctrl::pid_ctrl position_pid = robotun::ctrl::pid_ctrl(0.07, 0.0, 0.07, 0.0);
robotun::model::free_fall ff_model = robotun::model::free_fall();
std::vector<double> u_log;
/*******************************************************************************
**                      COMMON VARIABLE DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/


/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/


/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/

auto fig = matplot::figure(true);
matplot::axes_handle ax_;
matplot::line_handle line_;

void circle(double x, double y, double r)
{
	using namespace matplot;
	std::vector<double> theta = linspace(0, 2 * pi);
	std::vector<double> cx =
		transform(theta, [=](auto theta) { return r * cos(theta) + x; });
	std::vector<double> cy =
		transform(theta, [=](auto theta) { return r * sin(theta) + y; });
    line_ = ax_->plot(cx, cy);
}


std::vector<double> derivative(std::vector<double> state, int step, double t, double dt)
{
	std::vector<double> delta2;
    double dth = state[0], th = state[1], dphi = state[2], phi = state[3];

	double velocity_target = position_pid.get_ctrl(phi, dt);
	velocity_pid.set_target(velocity_target);
	double the_target = velocity_pid.get_ctrl(dphi, dt);
	th_pid.set_target(the_target);
	double U = -th_pid.get_ctrl(th, dt);

    U = U > 20.0 ? 20.0 : U;
    U = U < -20.0 ? -20.0 : U;
    u_log.push_back(U);

    delta2 = ff_model.get_delta2(state, U);
	std::cout << delta2[0] << "," << delta2[1] << std::endl;
    return { delta2[1], state[0], delta2[0], state[2]};
}

int main() {
    using namespace matplot;

    std::vector<double> times = linspace(0, 10, 500);
    robotun::ODE ode = robotun::ODE({ 0.0, pi/3, 0.0, 0.0 }, times, derivative);
    std::cout << "start\n";
    std::vector<std::vector<double>> solution = ode.solve();

    // Extract the state variables from the solution
    std::vector<double> w_solution;
    std::vector<double> theta_solution;
    std::vector<double> phi_solution;
    std::vector<double> u_solution;
    for (const auto& state : solution) 
    {
		w_solution.push_back(state[2]);
        theta_solution.push_back(state[1]);
        phi_solution.push_back(state[3]);
    }

    for (int i = 0; i < u_log.size(); i += 4)
    {
        u_solution.push_back(u_log[i]);
    }

    line_handle ph1 = plot(times, theta_solution);
	

    grid(true); // Changed to true for boolean
    show();
	
    //axis(equal);
    
    
    show();

    return 0;
}




/******************************** End of file *********************************/

