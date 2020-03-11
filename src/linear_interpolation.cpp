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

void linear_interp(VectorXd &X, MatrixXd &Y, double Xi, VectorXd &Yi ) {
	// Find the lower bounding index in X
	int Xj = 0;

    // This function does not extrapolate, clamp and check for ends
    if (Xi <= X[0]) {
        Xi = X[0];
        Xj = 0;
    } else if (Xi >= X[X.size()-1]) {
        Xi = X[X.size()-1];
        Xj = X.size()-1;
    } else {
        for (int i=0; i<X.size()-2; i++ ) {
            if ( Xi >= X[i] && Xi <= X[i+1] ) {
                Xj = i;
            }
        }
    }

    // Interpolate
    double Sj = (Xi - X[Xj]) / (X[Xj+1] - X[Xj]);
    Yi = Y.row(Xj) * (1.0 - Sj) + Y.row(Xj+1) * Sj;
}

} // namespace cassie_common_toolbox
