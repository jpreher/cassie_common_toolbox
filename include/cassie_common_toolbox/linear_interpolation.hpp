/*
 * @brief Linear interpolation tools.
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef LINEAR_INTERPOLATION_HPP
#define LINEAR_INTERPOLATION_HPP

#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;

namespace cassie_common_toolbox {

void linear_interp(VectorXd &X, MatrixXd &Y, double Xi, VectorXd &Yi );

} // namespace cassie_common_toolbox

#endif // LINEAR_INTERPOLATION_HPP
