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

class Body;

class BodyVec
{
public:

    //Body vector is wrapped in BodyVec class because the collision detector 
    //uses the body positions in the vector to link bodies to AABBs.
    //Body removal would mess this up (if not properly handled). 
    //Class wrapper prevents body removal. 
    //Note that the class only encapsulates the vector, and 
    //allows modification of bodies outsize of the class.
    //This is not totally fail-safe; bodies can still be invalidated 
    //by calling std::move on a non-const reference.

    BodyVec(int nBodies = 0);
    ~BodyVec();

    void addBody(Body& body);

    const std::vector<Body>& bodies() const { return m_bodies; }

    size_t size() const { return m_bodies.size(); }

    Body& at(int i) { return m_bodies[i]; }
    const Body& at(int i) const { return m_bodies[i]; }

    Body& operator[](int i) { return m_bodies[i]; }
    const Body& operator[](int i) const { return m_bodies[i]; }

private:

    std::vector<Body> m_bodies;
};