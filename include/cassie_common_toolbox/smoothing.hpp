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

class MovingAverage
{
public:
    MovingAverage();
    MovingAverage(int nSamples, int dim);
    void reset();
    void reconfigure(int nSamples, int dim);
    void update(Eigen::VectorXd &raw);
    Eigen::VectorXd getValue();
private:
    int nSamples;
    int cur_index;
    Eigen::MatrixXd array;
};

}

#endif // SMOOTHING_HPP
