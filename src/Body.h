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

#include <memory>
#include <vector>


#include "Vector2.h"
#include "Transform.h"
#include "AABB.h"

class Contact;
class Shape;

class Body
{
public:
    Body();
    Body(Transform trans, std::unique_ptr<Shape> shape, double density = 1.0);
  
    //Needed because of forward declared type in unique ptr.
    ~Body();
    Body(Body&& other);

    double               invMass()    const { return m_invMass; }
    double               invInertia() const { return m_invInertia; }
    const Transform&     transform()  const { return m_transform; }

    bool                         isStatic()             const;
    std::vector<Vector2>         globalPoints()         const;
    const std::vector<Vector2>&  localPoints()          const;
    AABB                         globalAABBWithMargin() const;

    //Compute contacts between this and the other bodies
    std::vector<Contact> contactProperties(const Body& otherBody) const;

    bool overlaps(Vector2 point) const; //Check if point is in body

    void updatePosition(Vector2 displacement, double rotation);

    //Used to enable mouse dragging of body
    void setPosition(Vector2 position);

    void setMargin(double margin);

    //Used to enable mouse dragging of body
    void hold() { m_held = true; }
    void release() { m_held = false; }

private:

    Transform  m_transform;
    double     m_invMass;
    double     m_invInertia;
    bool       m_held;

    std::unique_ptr<Shape> m_shape;

};