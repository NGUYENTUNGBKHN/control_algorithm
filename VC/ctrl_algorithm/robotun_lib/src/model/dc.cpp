/******************************************************************************/
/*! @addtogroup Group2
    @file       dc.cpp
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
#include "dc.h"
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
/**
 * @brief : DC simulation
 *      Equation of the motor:
 *          U = L * i' + R * i + k * w
 *          J = * w' + B * w = k * i - T1
 *          => w' = - w*(B + k_e * k_t/R)/J + k_t * U/(R * J)
 *      Where:
 *          U - Input volage
 *          L - Rotor indutance
 *          R - Rotor resistance
 *          J - Rotor moment of inertia
 *          B - Rotor friction
 *          T1 - external load torque
 * 
 *          k_e - back EMF constant, k_e * w - back EMF
 *          k_t - torque constant
 *      T1 = r * m * a = r^2 * w' * m, when r - shaft radius, m - load mass
 */         
DC::DC(double L_, double R_, double J_, double B_, double k_e_, double k_t_)
{
    L = L_;
    R = R_;
    J = J_;
    B = B_;
    k_e = k_e_;
    k_t = k_t_;
}

DC::~DC()
{

}

double DC::get_dw(double U, double w)
{
    double result = 0;

    result = -w*(B + k_e * k_t/R)/J + k_t * U / R / J;

    return (result);
}

/******************************** End of file *********************************/



