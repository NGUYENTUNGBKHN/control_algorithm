/******************************************************************************/
/*! @addtogroup Group2
    @file       ode.cpp
    @brief      
    @date       2025/09/12
    @author     Development Dept at Tokyo (nguyen-thanh-tung@jcm-hq.co.jp)
    @par        Revision
    $Id$
    @par        Copyright (C)
    Japan CashMachine Co, Limited. All rights reserved.
******************************************************************************/

/*******************************************************************************
**                                INCLUDES
*******************************************************************************/
#include "ode.h"
/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/


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

namespace robotun
{
ODE::ODE(std::vector<double> state_,
	std::vector<double> times_,
	derivative_t derivative_,
	ODE_STYLE style_
)
{
	state = state_;
	times = times_;
	derivative = derivative_;
	style = style_;
}

ODE::~ODE()
{
}

std::vector<double> ODE::integrate_euler(std::vector<double> state,
                                        int step, 
                                        double t,
                                        double dt,
                                        derivative_t derivative_func)
{
    std::vector<double> result;
    std::vector<double> k1 = derivative_func(state, step, t, dt);

	for (int i = 0; i < state.size(); i++) 
    {
		result.push_back(state[i] + k1[i] * dt);
	}
	return result;
}

std::vector<double> ODE::integrate_rk4(std::vector<double> state,
                                        int step, 
                                        double t,
                                        double dt,
                                        derivative_t derivative_func)
{
	std::vector<double> result;
    std::vector<double> k1, k2, k3, k4;
	std::vector<double> state_1, state_2, state_3;
    k1 = derivative_func(state, step, t, dt);
    for (int i = 0; i < state.size(); i++)
    {
		state_1.push_back(state[i] + k1[i] * dt / 2.0);
    }
    k2 = derivative_func(state_1, step, t + dt / 2, dt);
    for (int i = 0; i < state.size(); i++)
    {
        state_2.push_back(state[i] + k2[i] * dt / 2.0);
    }
    k3 = derivative_func(state_2, step, t + dt/2, dt);
    for (int i = 0; i < state.size(); i++)
    {
        state_3.push_back(state[i] + k3[i] * dt);
    }
    k4 = derivative_func(state_3, step, t + dt, dt);   
    for (int i = 0; i < state.size(); i++) 
    {
        double new_state = state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        result.push_back(new_state);
    }
    return result;
}

std::vector<std::vector<double>> ODE::solve()
{
    double dt = times[1] - times[0];
	std::vector<std::vector<double>> states;
	states.push_back(state);
    for (int i = 0; i < times.size(); i++)
    {
        states.push_back(integrate_rk4(states[states.size() - 1], i, times[i], dt, derivative));
    }
	states.erase(states.begin());
	return states;
}
}   /* end namespace robotun */


/******************************** End of file *********************************/

