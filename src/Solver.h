// MIT License

// Copyright (c) 2020 Marnix van den Berg <m.a.vdberg88@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <vector>

class Contact;
class BodyVec;
struct SolverConstraint;


struct SolverSettings 
{
    SolverSettings():
        maxDisplacement(0.0),
        maxRotation(0.0),
        maxPenetrationError(0.0),
        maxIterations(100),
        maxImpulseError(HUGE_VAL)
    {}

    double maxDisplacement;
    double maxRotation;
    double maxPenetrationError;
    double maxImpulseError;      //Set in solver
    int maxIterations;
};

class Solver
{
public:
    Solver();
    Solver::~Solver();

    void solveOverlaps(BodyVec* bodies, const std::vector<Contact>& contacts, const SolverSettings& solverSettings);

private:

    void initConstraints(const BodyVec& bodies, const std::vector<Contact>& contacts);
    void solve(BodyVec* bodies);
    void transformBodies(BodyVec* bodies);

    std::vector<SolverConstraint> m_solverConstraints;
    SolverSettings m_solverSettings;

};
