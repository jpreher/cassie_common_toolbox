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

#include <cassie_common_toolbox/motion_transition.hpp>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

namespace motion_transition {

void startTransition(PhaseVariable &phase, Eigen::VectorXd &ya, Eigen::VectorXd &dya, Eigen::MatrixXd &param) {
    int nParams = param.cols();
    int nOutputs = param.rows();

    VectorXi reserveParams(nParams - 2);
    VectorXi updateParam(2);
    updateParam << 0,1;
    for (int i=0; i<reserveParams.size(); ++i)
        reserveParams(i) = 2 + i;

    double tau_0 = 0.0;
    double dtau_0 = phase.dtau;

    VectorXd ya0(ya.size()), Lfya0(dya.size());
    ya0 << ya;
    Lfya0 << dya;

    MatrixXd Y(ya.size(), 2);
    Y << ya0, Lfya0;

    MatrixXd Phi_rem(2,2), a_bias(2,1), a_rem(2,1);
    MatrixXd a_temp(2,nParams), a_rest(1,nParams);
    a_temp.setZero();
    a_rest.setZero();

    a_temp(0,updateParam(0)) = 1;
    a_temp(1,updateParam(1)) = 1;

    VectorXd yd0(2), Lfyd0(2), yd0r(1), Lfyd0r(1);

    for (int i=0; i<nOutputs; ++i) {
        a_rest.setZero();

        for (int j=0; j<reserveParams.size(); ++j)
            a_rest(0,reserveParams(j)) = param(i,reserveParams(j));

        bezier_tools::bezier(a_rest, tau_0, yd0r);
        bezier_tools::dbezier(a_rest, tau_0, Lfyd0r);
        Lfyd0r = Lfyd0r * dtau_0;

        bezier_tools::bezier(a_temp, tau_0, yd0);
        bezier_tools::dbezier(a_temp, tau_0, Lfyd0);
        Lfyd0 = Lfyd0 * dtau_0;

        a_bias << yd0r, Lfyd0r;
        Phi_rem << yd0.transpose(), Lfyd0.transpose();

        a_rem = Phi_rem.inverse() * (Y.row(i).transpose() - a_bias);

        for (int j=0; j<updateParam.size(); ++j) {
            param(i,updateParam(j)) = a_rem(j);
        }
    }
}

void endTransition(PhaseVariable &phase, Eigen::VectorXd &ya, Eigen::MatrixXd &param) {
    int nParams = param.cols();
    int nOutputs = param.rows();

    VectorXi reserveParams(nParams - 2);
    VectorXi updateParam(2);
    updateParam << nParams-2, nParams-1;
    for (int i=0; i<reserveParams.size(); ++i)
        reserveParams(i) = i;

    double tau_0 = 1.0;
    double dtau_0 = phase.dtau;

    VectorXd dya(nOutputs);
    bezier_tools::dbezier(param, tau_0, dya);
    dya = dya * phase.dtau;

    VectorXd ya0(ya.size()), Lfya0(dya.size());
    ya0 << ya;
    Lfya0 << dya;

    MatrixXd Y(ya.size(), 2);
    Y << ya0, Lfya0;

    MatrixXd Phi_rem(2,2), a_bias(2,1), a_rem(2,1);
    MatrixXd a_temp(2,nParams), a_rest(1,nParams);
    a_temp.setZero();
    a_rest.setZero();

    a_temp(0,updateParam(0)) = 1;
    a_temp(1,updateParam(1)) = 1;

    VectorXd yd0(2), Lfyd0(2), yd0r(1), Lfyd0r(1);

    for (int i=0; i<nOutputs; ++i) {
        a_rest.setZero();

        for (int j=0; j<reserveParams.size(); ++j)
            a_rest(0,reserveParams(j)) = param(i,reserveParams(j));

        bezier_tools::bezier(a_rest, tau_0, yd0r);
        bezier_tools::dbezier(a_rest, tau_0, Lfyd0r);
        Lfyd0r = Lfyd0r * dtau_0;

        bezier_tools::bezier(a_temp, tau_0, yd0);
        bezier_tools::dbezier(a_temp, tau_0, Lfyd0);
        Lfyd0 = Lfyd0 * dtau_0;

        a_bias << yd0r, Lfyd0r;
        Phi_rem << yd0.transpose(), Lfyd0.transpose();

        a_rem = Phi_rem.inverse() * (Y.row(i).transpose() - a_bias);

        for (int j=0; j<updateParam.size(); ++j) {
            param(i,updateParam(j)) = a_rem(j);
        }
    }
}

}


// POWELL Method

