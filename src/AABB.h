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

#pragma once

#include "Vector2.h"

class AABB
{
public:

    AABB();
    AABB(Vector2 lowerBound, Vector2 upperBound);

    double perimiter() const { return m_perimiter; }
    Vector2 lowerBound() const { return m_lowerBound; }
    Vector2 upperBound() const { return m_upperBound; }

    void   merge(const AABB& aabb0, const AABB& aabb1);
    bool contains(const AABB& aabb) const;
    bool overlaps(const AABB& aabb) const;

private:

    double computePerimiter() const;

    Vector2 m_lowerBound;
    Vector2 m_upperBound;
    double  m_perimiter;
};

