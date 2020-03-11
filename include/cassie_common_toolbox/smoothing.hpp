/*
 * @brief A general filter class. Replicates "filt" in Matlab
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef SMOOTHING_HPP
#define SMOOTHING_HPP

#include <Eigen/Dense>
#include <iostream>

namespace cassie_common_toolbox {

class ButterFilter
{
public:
    ButterFilter();
    ButterFilter(Eigen::VectorXd &b, Eigen::VectorXd &a);
    void reconfigure(Eigen::VectorXd &b, Eigen::VectorXd &a);
    void update(double x_raw);
    double getValue();
    void reset();

private:
    long nb;
    long na;
    Eigen::VectorXd y_; // Array of previously filtered values
    Eigen::VectorXd x_; // Array of raw values
    Eigen::VectorXd b_; // filter coefficient
    Eigen::VectorXd a_; // filter coefficient
};

}

#endif // SMOOTHING_HPP
