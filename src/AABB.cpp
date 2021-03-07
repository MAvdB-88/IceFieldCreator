// MIT License

// Copyright(c) 2019 Erin Catto http ://www.box2d.org
// --Modified based on box2d source code--
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

#include "AABB.h"
#include <algorithm>

AABB::AABB() :
    m_lowerBound(0.0, 0.0),
    m_upperBound(0.0, 0.0),
    m_perimiter(0.0)
{}


AABB::AABB(Vector2 lowerBound, Vector2 upperBound) :
    m_lowerBound(lowerBound), m_upperBound(upperBound)
{
    m_perimiter = computePerimiter();
}


double AABB::computePerimiter() const
{
    double wx = m_upperBound[0] - m_lowerBound[0];
    double wy = m_upperBound[1] - m_lowerBound[1];
    return 2.0 * (wx + wy);
}

void AABB::merge(const AABB& aabb1, const AABB& aabb2)
{

    for (int i = 0; i < 2; i++)
    {
        m_lowerBound[i] = std::min(aabb1.m_lowerBound[i], aabb2.m_lowerBound[i]);
        m_upperBound[i] = std::max(aabb1.m_upperBound[i], aabb2.m_upperBound[i]);
    }
}

bool AABB::contains(const AABB& aabb) const
{
    for (int i = 0; i < 2; i++)
    {
        if (m_lowerBound[i] < aabb.m_lowerBound[i]) return false;
        if (m_upperBound[i] > aabb.m_upperBound[i]) return false;
    }

    return true;
}

bool AABB::overlaps(const AABB& aabb) const
{
    return !(aabb.m_upperBound[0] < m_lowerBound[0]
        || aabb.m_lowerBound[0] > m_upperBound[0]
        || aabb.m_upperBound[1] < m_lowerBound[1]
        || aabb.m_lowerBound[1] > m_upperBound[1]
        );
}
