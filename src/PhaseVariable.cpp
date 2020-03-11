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

#include <cassie_common_toolbox/PhaseVariable.hpp>
#include <control_utilities/limits.hpp>

PhaseVariable::PhaseVariable() {
    this->tau = 0.0;
    this->dtau = 0.0;
    this->pActual = 0.0;
    this->dpActual = 0.0;
    this->phaseRange << 0.0, 1.0;
}

void PhaseVariable::reconfigure(Eigen::Vector2d &phaseRange, double timeScale=1.0) {
    this->phaseRange << phaseRange;
    this->timeScale = timeScale;
}

void PhaseVariable::update(double time) {
    this->calcP(time);
    this->calcTau();
    this->calcDTau();
}

void PhaseVariable::calcP(double time) {
    this->pActual = time * this->timeScale;
    this->dpActual = this->timeScale;
}

void PhaseVariable::calcTau() {
    this->tau = (this->pActual - this->phaseRange(0)) / (this->phaseRange(1) - this->phaseRange(0));
    this->tau = control_utilities::clamp(this->tau, 0.0, 1.25);
}

void PhaseVariable::calcDTau() {
    this->dtau = this->dpActual / (this->phaseRange(1) - this->phaseRange(0));
}

Eigen::Vector2d PhaseVariable::getPhaseRange() {
    return this->phaseRange;
}
