#ifndef __GRAV_COMP_H__
#define __GRAV_COMP_H__

#include "Eigen/Dense"

// q is a 8x1 vector of joint positions (q_col, q_1, q_2, ...)
// result is the 8x1 vector of gravity compensation forces (tau_g_col, tau_g_1, tau_g_2, ...)
void computeGravComp(const Eigen::VectorXd& q, Eigen::VectorXd& result);

#endif  // __GRAV_COMP_H__
