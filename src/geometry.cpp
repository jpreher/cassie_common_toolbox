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

#include <cassie_common_toolbox/geometry.hpp>

void eulerZYX(Eigen::Quaterniond &q, Eigen::EulerAnglesZYXd &euler){
    double q0, q1, q2, q3;

    q0 = q.w();
    q1 = q.x();
    q2 = q.y();
    q3 = q.z();

    euler.alpha() = atan2(-2.0*q1*q2 + 2.0*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
    euler.beta()  = asin(2.0*q1*q3 + 2.0*q0*q2);
    euler.gamma() = atan2(-2.0*q2*q3 + 2.0*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
}

void eulerXYZ(Eigen::Quaterniond &q, Eigen::EulerAnglesXYZd &euler){
    double q0, q1, q2, q3;

    q0 = q.w();
    q1 = q.x();
    q2 = q.y();
    q3 = q.z();

    euler.alpha() = atan2(2.0*q0*q1 + 2.0*q2*q3, 1.0 - 2.0 * (q1*q1 + q2*q2)); // q1*q1 + q0*q0 - q3*q3 - q2*q2);
    euler.beta()  = asin(2.0 * ( q0*q2 - q3*q1 ));
    euler.gamma() = atan2(2.0 * ( q0*q3 + q1*q2 ), 1.0 - 2.0 * (q2*q2+q3*q3));
}

void eulerXYZ(Eigen::Matrix3d &R, Eigen::EulerAnglesXYZd &euler){
    double y1, y2, x1, x2, z1, z2, x, y, z;
    if ( abs(abs(R(2,0)) - 1.0) > 0.00000001 ) {
        y1 = -asin(R(2,0));
        //y2 = 3.14159265359 - y1;

        x1 = atan2(R(2,1)/cos(y1), R(2,2)/cos(y1));
        //x2 = atan2(R(2,1)/cos(y2), R(2,2)/cos(y2));

        z1 = atan2(R(1,0)/cos(y1), R(0,0)/cos(y1));
        //z2 = atan2(R(1,0)/cos(y2), R(0,0)/cos(y2));

        euler.alpha() = x1;
        euler.beta()  = y1;
        euler.gamma() = z1;
    } else {
        z = 0.0;
        if ( abs(R(2,0) + 1.0) < 0.00000001 ) {
            y = 3.14159265359/2.0;
            x = z + atan2(R(0,1), R(0,2));
        } else {
            y = -3.14159265359/2.0;
            x = -z + atan2(-R(0,1), -R(0,2));
        }
        euler.alpha() = x;
        euler.beta()  = y;
        euler.gamma() = z;
    }
}

Eigen::Matrix3d skew(Eigen::Vector3d &v) {
    Eigen::Matrix3d sk;

    sk << 0.0, -v(2), v(1),
          v(2), 0.0, -v(0),
         -v(1), v(0), 0.0;
    return sk;
}
