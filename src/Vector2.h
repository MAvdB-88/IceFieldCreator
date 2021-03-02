// GNU License

// Copyright (C) 2011-2013 Ceetron AS
//--Modified based on Custom Visualization Core library Source code--
// Copyright (c) 2020 Marnix van den Berg <m.a.vdberg88@gmail.com>

//   GNU General Public License Usage
//   This is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.



#pragma once

#include <cmath>

class Vector2
{
public:
    Vector2()                                             { m_v[0] = 0; m_v[1] = 0; }
    Vector2(double x, double y)                           { m_v[0] = x; m_v[1] = y; }
    Vector2(const Vector2& other)                         { *this = other; }

    Vector2&        operator=(const Vector2& rhs)         { m_v[0] = rhs.m_v[0]; m_v[1] = rhs.m_v[1]; return *this; }

    const double& x() const                               { return m_v[0]; }  
    const double& y() const                               { return m_v[1]; }  
    double&       x()                                     { return m_v[0]; }  
    double&       y()                                     { return m_v[1]; }  
    const double& getX() const                            { return m_v[0]; }
    const double& getY() const                            { return m_v[1]; }
    void          setX(double xVal)                       { m_v[0] = xVal; }
    void          setY(double yVal)                       { m_v[1] = yVal; }
    void          setZero()                               { m_v[0] = 0.0; m_v[1] = 0.0; }

    const Vector2   operator+(const Vector2& rhs) const   { return Vector2(m_v[0] + rhs.m_v[0], m_v[1] + rhs.m_v[1]); }
    const Vector2   operator-(const Vector2& rhs) const   { return Vector2(m_v[0] - rhs.m_v[0], m_v[1] - rhs.m_v[1]); }
    Vector2&        operator+=(const Vector2& rhs)        { m_v[0] += rhs.x(); m_v[1] += rhs.y(); return *this; }
    Vector2&        operator-=(const Vector2& rhs)        { m_v[0] -= rhs.x(); m_v[1] -= rhs.y(); return *this; }
    const Vector2   operator-() const                     { return Vector2(-m_v[0], -m_v[1]); }

    const Vector2   operator/(double scalar) const        { return Vector2(m_v[0]/scalar, m_v[1]/scalar); }
    Vector2&        operator*=(double scalar)             { m_v[0] *= scalar; m_v[1] *= scalar; return *this; }
    Vector2&        operator/=(double scalar)             { m_v[0] /= scalar; m_v[1] /= scalar; return *this; }

    double        dot(const Vector2& rhs) const           { return m_v[0]*rhs.m_v[0] + m_v[1]*rhs.m_v[1]; }
    double        length2() const                         { return m_v[0]*m_v[0] + m_v[1]*m_v[1]; }
    double        length() const                          { return std::sqrt(length2()); }
    Vector2       normalized() const                      { double lnth = length(); if (lnth != 0) return (*this) / lnth; else  return Vector2( 0.0, 0.0); }
	void          normalize()                             { double lnth = length(); if (lnth != 0) *this/= lnth; }
    Vector2       perpendicular() const                   { return Vector2(-y(), x()); }
    Vector2       rotated(double angleRad) const          { double s = std::sin(angleRad); double c = std::cos(angleRad); 
                                                              return Vector2( c * x() - s * y(), s * x() + c * y()); }

    // Equivalent to 3D cross product with z = 0 in both vectors. The result is the magnitude of the z in the resulting vector
    double        cross(const Vector2& rhs) const         { return x() * rhs.y() - y() * rhs.x(); } 

    double&       operator[](int i)                        { return m_v[i]; }
    const double& operator[](int i) const { return m_v[i]; }

private:
    double m_v[2];
};


inline const Vector2 operator*(const Vector2& v, double s)     { return Vector2(s*v.x(), s*v.y()); }
inline const Vector2 operator*(double s, const Vector2& v)     { return Vector2(s*v.x(), s*v.y()); }
