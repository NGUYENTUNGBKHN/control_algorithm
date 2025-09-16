/******************************************************************************/
/*! @addtogroup Group2
    @file       free_fall.cpp
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
#include <model.h>
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
    namespace model
    {
        free_fall::free_fall(double M_,
                        double m_,
                        double r_ ,
                        double l_,
                        double g_,
                        double b1_ ,
                        double b2_ )
        {
            M = M_; 
            m = m_;
            r = r_; 
            l = l_;
            g = g_;
            b1 = b1_; 
            b2 = b2_;
            I = 0.5 * M * r;
        }
        
        free_fall::~free_fall()
        {
        }

        std::vector<double> free_fall::get_delta2(std::vector<double> state, double u)
        {
			//std::vector<double> result = { 0.0, 0.0 };

			double dth = state[0], th = state[1], dphi = state[2], phi = state[3];
			double s = sin(th), c = cos(th);

            double _dphi = (m * r * (l * pow(dth, 2) * s + b1 * dth * c - g * s * c) - b2 * dphi + u)/(I + m * pow(r, 2) * pow(s, 2));
            double _dth = (g * s - r * _dphi * c - b1 * dth) / l;

			return { _dphi, _dth };
        }
        
    } // namespace model
    
} // namespace robotun







/******************************** End of file *********************************/


