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

#include <cassie_common_toolbox/linear_interpolation.hpp>

namespace cassie_common_toolbox {


int find_index(VectorXd &X, double &Xi) {
    // Find the lower bounding index in X
    int Xj = 0;

    // This function does not extrapolate, clamp and check for ends
    if (Xi <= X[0]) {
        Xi = X[0];
        Xj = 0;
    } else if (Xi >= X[X.size()-1]) {
        Xi = X[X.size()-2];
        Xj = X.size()-3;
    } else if (Xi >= X[X.size()-2]) {
            Xj = X.size()-2;
    } else {
        for (int i=0; i<X.size()-2; i++ ) {
            if ( Xi >= X[i] && Xi <= X[i+1] ) {
                Xj = i;
            }
        }
    }
    return Xj;
}

void linear_interp(VectorXd &X, MatrixXd &Y, double Xi, VectorXd &Yi ) {
    int Xj = 0;
    Xj = find_index(X, Xi);

    // Interpolate
    double Sj = (Xi - X[Xj]) / (X[Xj+1] - X[Xj]);
    Yi = Y.row(Xj) * (1.0 - Sj) + Y.row(Xj+1) * Sj;
}


void bilinear_interp(VectorXd &X, VectorXd &Y, std::vector< std::vector<VectorXd> > Z, double Xi, double Yi, VectorXd &Zi ) {
    // Find indices
    int Xj = 0;
    Xj = find_index(X, Xi);

    int Yj = 0;
    Yj = find_index(Y, Yi);

    // X direction
    double SXj = (Xi - X[Xj]) / (X[Xj+1] - X[Xj]);
    double SYj = (Yi - Y[Yj]) / (Y[Yj+1] - Y[Yj]);

    // Z_x_y1 = Z[Xj][Yj] * (1.0 - SXj) + Z[Xj+1][Yj] * SXj;
    // Z_x_y2 = Z[Xj][Yj+1] * (1.0 - SXj) + Z[Xj+1][Yj+1] * SXj;
    Zi = ( Z[Xj][Yj] * (1.0 - SXj) + Z[Xj+1][Yj] * SXj ) * (1.0 - SYj)
            + ( Z[Xj][Yj+1] * (1.0 - SXj) + Z[Xj+1][Yj+1] * SXj ) * SYj;
}






} // namespace cassie_common_toolbox
