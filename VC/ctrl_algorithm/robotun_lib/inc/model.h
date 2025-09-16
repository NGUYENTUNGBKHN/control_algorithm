#pragma once

#ifndef _MODEL_H_
#define _MODEL_H_
#include <vector>
/* CODE */
namespace robotun 
{
	namespace model
	{
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

		class free_fall
		{
		private:
			/* data */
			double M, m, r, l, g, b1, b2, I;
		public:
			free_fall(double M_ = 0.25,
					double m_ = 0.3,
					double r_ = 0.25,
					double l_ = 1.0,
					double g_ = 9.8,
					double b1_ = 0.01, 
					double b2_ = 0.01);
			~free_fall();

			std::vector<double> get_delta2(std::vector<double> state, double u = 0.0);
		};
	}
}

#endif



