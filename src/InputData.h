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
#include "Vector2.h"

struct Settings
{
    //If not provided in input file, these default values will be used
    Settings():
        maxCollisionMargin(0.05),
        maxResidualPenetration(1.0e-3),
        maxSolverIterations(1000)
    {}

    //Margins and settings that can be used to improve convergence.

    double              maxCollisionMargin;        //Body dimension are extended by the collision margin, input is maximum, can be reduces during sim by sliders
    double              maxResidualPenetration;    //If all body penetrations are below this threshold value, output is produced and the program stops.
    int                 maxSolverIterations;       //Maximum number of iterations in impulse solver.
};

struct InputData
{

    InputData():
        domainSize(5.0,5.0),
        concentration(0.0)
    {}

    ~InputData() {}

    Vector2                            domainSize;     //Rectangular boundaries
    std::vector<std::vector<Vector2>>  bodyPointsVec;  //point defining body geometries
    double                             concentration;  //aerial percentage of domain covered by polygons.
    Settings                           settings;
};