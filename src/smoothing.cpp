/*
 * MIT License
 * 
 * Copyright (c) 2020 Jenna Reher (jreher@caltech.edu)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <cassie_common_toolbox/smoothing.hpp>

namespace cassie_common_toolbox {

ButterFilter::ButterFilter(){
    Eigen::VectorXd a(1);
    Eigen::VectorXd b(1);

    // This filter does nothing. Produce the raw value.
    a(0) = 1.;
    b(0) = 1.;
    this->reconfigure(b,a);
}

ButterFilter::ButterFilter(Eigen::VectorXd &b, Eigen::VectorXd &a) {
    this->reconfigure(b,a);
}

void ButterFilter::reconfigure(Eigen::VectorXd &b, Eigen::VectorXd &a) {
    this->nb = b.size();
    this->na = a.size();
    this->b_ = b;
    this->a_ = a;
    this->y_.resize(na);
    this->y_.setZero();
    this->x_.resize(nb);
    this->x_.setZero();
}

void ButterFilter::reset() {
    this->y_.setZero();
    this->x_.setZero();
}

void ButterFilter::update(double x_raw) {
    // Shift the data back and append the raw
    for (long i=nb-1; i>0; i--) {
        x_(i) = x_(i-1);
    }
    for (long i=na-1; i>0; i--) {
        y_(i) = y_(i-1);
    }
    x_(0) = x_raw;

    // Get the solution -- MATLAB pseudocode...
    // a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
    //                         - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
    double y = 0.;
    for (long i=0; i<nb; i++) {
        y += b_(i) * x_(i);
    }
    for(long i=1; i<na; i++) {
        y -= a_(i) * y_(i);
    }

    // Multiply solution... a(0) should be 1 from MATLAB
    y *= a_(0);
    y_(0) = y;
}

double ButterFilter::getValue() {
    return y_(0);
}


}
