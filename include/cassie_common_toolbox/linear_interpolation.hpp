/*
 * @brief Linear interpolation tools.
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef LINEAR_INTERPOLATION_HPP
#define LINEAR_INTERPOLATION_HPP

#include <Eigen/Dense>
#include <iostream>
#include <vector>
using namespace Eigen;

namespace cassie_common_toolbox {

int find_index(VectorXd &X, double &Xi);

void linear_interp(VectorXd &X, MatrixXd &Y, double Xi, VectorXd &Yi );

void bilinear_interp(VectorXd &X, VectorXd &Y, std::vector< std::vector<VectorXd> > Z, double Xi, double Yi, VectorXd &Zi );





} // namespace cassie_common_toolbox

#endif // LINEAR_INTERPOLATION_HPP
