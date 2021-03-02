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
#include "Orientation.h"

class Transform
{
public:

    Transform() :
        m_position(0.0,0.0),
        m_orientation()
    {}

    Transform(Vector2 position, Orientation orientation):
        m_position(position),
        m_orientation(orientation)
    {}

    Transform(Vector2 position, double angle) :
        m_position(position),
        m_orientation(angle)
    {}

    void multiply(Vector2 translation, double rotation)
    {
        m_position += translation;
        Orientation or (rotation);
        m_orientation = m_orientation * or;

    }

    Orientation orientation() const { return m_orientation; }
    Vector2 position() const { return m_position; }

    void setPosition(Vector2 position) { m_position = position; }

private:
    Vector2      m_position;
    Orientation  m_orientation;
};


inline const Vector2 transformVec(Transform trans, Vector2 vec)
{
    return rotateVec(trans.orientation(), vec) + trans.position();
};


inline const Vector2 transformVecInv(Transform trans, Vector2 vec)
{
    Vector2 p = vec - trans.position();
    return rotateVecInv(trans.orientation(), p);
};

inline Transform inverseTimes(const Transform& A, const Transform& B)
{
    //Complete this tomorrow. Should work.
    Orientation orientation = inverseTimes(A.orientation(), B.orientation());
    Vector2 position = rotateVecInv(A.orientation(), B.position() - A.position());

    return Transform(position, orientation);
}