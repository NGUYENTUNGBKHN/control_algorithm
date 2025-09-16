#include <cmath>
#include <matplot/matplot.h>
#include <model.h>
#include <ctrl.h>
#include <ode.h>

std::vector<double> u_log;

robotun::model::DC dc = robotun::model::DC();
robotun::ctrl::pid_ctrl velocity_pid = robotun::ctrl::pid_ctrl(20.0, -0.1, 0.1, 0.0);
robotun::ctrl::pid_ctrl position_pid = robotun::ctrl::pid_ctrl(2.0, 0.0, 0.0, 1.0);

std::vector<double> derivative(std::vector<double> state, int step, double t, double dt)
{
    std::vector<double> result;
    double w = state[0];
    double theta = state[1];

    double w_target = position_pid.get_ctrl(theta, dt);
	velocity_pid.set_target(w_target);
    double U = velocity_pid.get_ctrl(w, dt);


    U = U > 12.0 ? 12.0 : U;
	U = U < -12.0 ? -12.0 : U;
	u_log.push_back(U);

	return { dc.get_dw(U, w), w };
}

int main() {
    using namespace matplot;

    std::vector<double> times = linspace(0, 5, 5000);
    robotun::ODE ode = robotun::ODE({ 0.0, 0.0 }, times, derivative);

    std::vector<std::vector<double>> solution = ode.solve();

    // Extract the state variables from the solution
    std::vector<double> w_solution;
    std::vector<double> theta_solution;
	std::vector<double> u_solution;
    for (const auto& state : solution) {
        if (state.size() >= 2) {
            w_solution.push_back(state[0]);
            theta_solution.push_back(state[1]);
        }
    }

    for (int i = 0; i < u_log.size(); i += 4)
    {
		u_solution.push_back(u_log[i]);
    }

    // Now plot the extracted data
    line_handle ph1 = plot(times, w_solution);
    hold(on);
    line_handle ph2 = plot(times, theta_solution);
    line_handle ph3 = plot(times, u_solution);

    // Create a vector of handles for the legend
    std::vector<matplot::axes_object_handle> lines = { ph1, ph2, ph3 };

    legend(lines, std::vector<std::string>{"w", "theta", "U"});
	
    grid(true); // Changed to true for boolean
    show();

    return 0;
}



