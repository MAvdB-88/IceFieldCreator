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

#include "Vector2.h"

//Data structure used in solver. All are defined in world coordinates.
struct SolverConstraint
{
    SolverConstraint()
        :effectiveMass(0.0),
        normal0(0.0, 0.0),
        normal1(0.0, 0.0),
        relPos0CrossNormal(0.0),
        relPos1CrossNormal(0.0),
        angularComponent0(0.0),
        angularComponent1(0.0),
        linearComponent0(0.0,0.0),
        linearComponent1(0.0,0.0),
        impulse(0.0),
        rhs(0.0),
        bodyIdx0(-1),
        bodyIdx1(-1)
    {}

    double effectiveMass;      //Inverse of relative velocity change in normal direction at the contact point caused by a unit impulse
    Vector2 normal0;           //normal pointing towards body 0
    Vector2 normal1;           //normal1 = -normal0, pointing towards body 1
    double relPos0CrossNormal; //cross product (2D equivalnt) of the relatice contact point position and the contact normal, body0.
    double relPos1CrossNormal; //cross product (2D equivalnt) of the relatice contact point position and the contact normal, body1. 
    double angularComponent0;  //Rotational velocity change caused by a unit impulse at the contact, body 0.
    double angularComponent1;  //Rotational velocity change caused by a unit impulse at the contact, body 1.
    Vector2 linearComponent0;  //Linear velocity change caused by a unit impulse at the contact, body 0.
    Vector2 linearComponent1;  //Linear velocity change caused by a unit impulse at the contact, body 1.
    double impulse;            //Impulse applied at this contact.

    //Penalty term, defined here as the penetration error * effective mass. 
    //This definition leads to a body displacement equal to the penetration 
    //when considering the velocity change of the body caused by the applied 
    //contact impulses and integrating the body positions over a unit time interval.
    //Note that the current code does not actually simulate dynamics and does not store the body velocities,
    //as penetration correction is the purpose. However, exactly the same data structure can be used for a dynamics,
    //by storing body velocity data and defining the penalty term based on a velocity error.
    double rhs;                

    int bodyIdx0; //-1 means uninitialized
    int bodyIdx1; //-1 means uninitialized
};