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

class Transform;

class Contact
{
public:
    Contact();

    Vector2 contactPoint() const { return m_contactPoint; }
    Vector2 normal()       const { return m_normal; }
    double  penetration()  const { return m_penetration; }
    int     bodyIdx0()     const { return m_bodyIdx0; }
    int     bodyIdx1()     const { return m_bodyIdx1; }

    void setBodyIdx0(int idx0)                 { m_bodyIdx0 = idx0; }
    void setBodyIdx1(int idx1)                 { m_bodyIdx1 = idx1; }
    void setContactPoint(Vector2 contactPoint) { m_contactPoint = contactPoint; }
    void setNormal(Vector2 normal)             { m_normal = normal; }
    void setPenetration(double penetration)    { m_penetration = penetration; }

    void transform(const Transform& trans);

private:

    int m_bodyIdx0; //Index of the first contacting body in the body vector. -1 means uninitialized.
    int m_bodyIdx1; //Index of the second contacting body in the body vector. -1 means uninitialized.

    Vector2 m_contactPoint;
    Vector2 m_normal;
    double  m_penetration;
};