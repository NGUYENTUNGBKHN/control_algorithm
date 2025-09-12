#pragma once

class DC
{
public:
	DC(double L_ = 0.01, double R_ = 4.0, double J_ = 0.3, double B_ = 1e-2, double k_e_ = 1.0, double k_t_ = 1.0);
	//DC(float L_, float R_, float J_, float B_, float k_e_, float k_t_);
	~DC();

	double get_dw(double U, double w);
private:
	double L, R, J, B, k_e, k_t;
};

