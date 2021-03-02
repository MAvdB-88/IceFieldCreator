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

#include "Body.h"
#include "Contact.h"

#include "ErrorLogger.h"

Body::Body() :
    m_invMass(0.0),
    m_invInertia(0.0),
    m_linearVelocityChangeImpulse(0.0, 0.0),
    m_angularVelocityChangeImpulse(0.0),
    m_held(false)
{}

Body::Body(Transform trans, Shape& shape, double density):
    m_transform(trans),
    m_linearVelocityChangeImpulse(0.0,0.0),
    m_angularVelocityChangeImpulse(0.0),
    m_shape(std::move(shape)),
    m_invMass(0.0),
    m_invInertia(0.0),
    m_held(false)
{
    //It is found that overlap resolving works best with mass and inertia properties proportional to the earial properties.
    //Resolving overlaps would also work for other mass and interia properties, but generally the convergence is slower.

    //TODO: Check if these can fail or will always work.
    double mass = m_shape.calcArea()*density;
    if (mass > DBL_EPSILON) m_invMass = 1.0 / mass;

    double inertia = m_shape.calcInertia()*density;
    if (mass > DBL_EPSILON) m_invInertia = 1.0 / inertia;
}


bool Body::isStatic() const
{
    if (m_invMass < DBL_EPSILON) return true;
    else if (m_held) return true;
    else return false;
}


AABB Body::globalAABBWithMargin() const
{
    return m_shape.calcTransformedAABBWithMargin(m_transform);
}

std::vector<Vector2> Body::globalPoints() const
{
    return m_shape.transformedPoints(m_transform);
}

const std::vector<Vector2>& Body::localPoints() const
{
    return m_shape.localPoints();
}


bool Body::overlaps(Vector2 point) const
{
    Vector2 locPoint = transformVecInv(m_transform, point);
    return m_shape.overlaps(locPoint);
}


void Body::updatePosition(double maxDisplacement, double maxRotation)
{
    //Positions are updated assuming a unit time step and application of impulses at the beginning of the time step.
    //Velocity data is not stored.

    for (int i = 0; i < 2; i++)
    {
        if (std::abs(m_linearVelocityChangeImpulse[i]) > maxDisplacement)
        {
            double dispX = m_linearVelocityChangeImpulse[i];
            double dispConstr = dispX * maxDisplacement / std::abs(dispX);
            m_linearVelocityChangeImpulse[i] = dispConstr;
        }
    }

    if (std::abs(m_angularVelocityChangeImpulse) > maxRotation)
    {
        double rotConstr = m_angularVelocityChangeImpulse * maxRotation 
            / std::abs(m_angularVelocityChangeImpulse);
        m_angularVelocityChangeImpulse = rotConstr;
    }


    if (!isStatic()) //mass = 0 indicates a static body
    {
        m_transform.multiply(m_linearVelocityChangeImpulse, m_angularVelocityChangeImpulse);
    }

    m_linearVelocityChangeImpulse.setZero();
    m_angularVelocityChangeImpulse = 0.0;
}


void Body::setPosition(Vector2 position)
{
    m_transform.setPosition(position);
}


void Body::setMargin(double margin)
{
    m_shape.setCollisionMargin(margin);
}



std::vector<Contact> Body::contactProperties(const Body& otherBody) const
{
    //Contact properties are calculated in the coordinates of this body, 
    //such that only the shape of the other body needs to be transformed.
    const Transform& otherTrans = otherBody.m_transform;
    const Shape& otherShape = otherBody.m_shape;

    //the combined transform transforms the shape of the other body from the other body's coordinate system
    //to this bodies coordinate system.
    Transform combinedTransform = inverseTimes(m_transform, otherTrans);

    //Overlap properties determination can fail in specific cases, 
    //but these isolated will be corrected in subsequent steps. No need for warnings/errors.
    std::vector<Contact> contacts;
    m_shape.overlap(otherShape, combinedTransform, &contacts);

    //Transform the contact to world coorinates.
    for (int i = 0; i < contacts.size(); i++)
    {
        contacts[i].transform(m_transform);
    }

    return  contacts;
}