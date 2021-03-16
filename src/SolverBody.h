#pragma once

#include "Vector2.h"

class SolverBody
{
public:

    SolverBody():
        m_angularVelocityChangeImpulse(0.0)
    {}

    inline void setZero()
    {
        m_linearVelocityChangeImpulse.setZero();
        m_angularVelocityChangeImpulse = 0.0;
    }

    inline const Vector2& linearVelocityChangeImpulse() { return m_linearVelocityChangeImpulse; }
    inline const double   angularVelocityChangeImpulse() { return m_angularVelocityChangeImpulse; }

    inline void applyImpulse(const Vector2& linearComponent, double angularComponent, double impulseMagnitude)
    {
        m_linearVelocityChangeImpulse += linearComponent * impulseMagnitude;
        m_angularVelocityChangeImpulse += angularComponent * impulseMagnitude;
    }

private:

    Vector2    m_linearVelocityChangeImpulse;
    double     m_angularVelocityChangeImpulse;
};
