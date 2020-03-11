/*
 * @brief Smoothly update a Bezier polynomial while preserving endpoint velocity.
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef CASSIE_MOTION_TRANSITION_HPP
#define CASSIE_MOTION_TRANSITION_HPP

#include <cassie_common_toolbox/bezier_tools.hpp>
#include <cassie_common_toolbox/PhaseVariable.hpp>

namespace motion_transition {

void startTransition(PhaseVariable &phase, Eigen::VectorXd &ya, Eigen::VectorXd &dya, Eigen::MatrixXd &param);
void endTransition(PhaseVariable &phase, Eigen::VectorXd &ydEnd, Eigen::MatrixXd &param);
void motionTransition();

}

#endif // CASSIE_MOTION_TRANSITION_HPP