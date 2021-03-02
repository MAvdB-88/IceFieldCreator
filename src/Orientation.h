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

#include <cmath>
#include "Vector2.h"


class Orientation
{
public:

    Orientation():
        m_sinAngle(0.0),
        m_cosAngle(1.0)
    {}

    Orientation(double angle)
    {
        m_sinAngle = std::sin(angle);
        m_cosAngle = std::cos(angle);
    }

    Orientation(double sinAngle, double cosAngle) :
        m_sinAngle(sinAngle),
        m_cosAngle(cosAngle)
    {}

    void set(double angle)
    {
        m_sinAngle = std::sin(angle);
        m_cosAngle = std::cos(angle);
    }

    double angle() const 
    {
        return std::atan2(m_sinAngle, m_cosAngle);
    }

    double sin() const { return m_sinAngle; };
    double cos() const { return m_cosAngle; };

private:

    double m_sinAngle;
    double m_cosAngle;
};


inline const Vector2 rotateVec(Orientation or, Vector2 vec)
{
    double x = (or.cos() * vec.x() - or.sin() * vec.y());
    double y = (or.sin() * vec.x() + or.cos() * vec.y());

    return Vector2(x, y);
}


inline const Vector2 rotateVecInv(Orientation or , Vector2 vec)
{
    double x = (or .cos() * vec.x() + or .sin() * vec.y());
    double y = (-or .sin() * vec.x() + or .cos() * vec.y());

    return Vector2(x, y);
}

inline const Orientation operator*(const Orientation& or0 , const Orientation& or1) 
{    
    double sin = or0.sin() * or1.cos() + or0.cos() * or1.sin();
    double cos = or0.cos() * or1.cos() - or0.sin() * or1.sin();

    return Orientation(sin,cos);
}

inline const Orientation inverseTimes(const Orientation& or0, const Orientation& or1) 
{
    double sin = or0.cos() * or1.sin() - or0.sin() * or1.cos();
    double cos = or0.cos() * or1.cos() + or0.sin() * or1.sin();

    return Orientation(sin, cos);
}