// MIT License

// Copyright (c) 2021 Marnix van den Berg <m.a.vdberg88@gmail.com>

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
#include <array>
#include "Vector2.h"

class Transform;
class Contact;
class AABB;

struct Line
{
    Line(Vector2 dir, double dist) :
        dir(dir),
        dist(dist)
    {}

    Line() :
        dir(0.0, 0.0),
        dist(0.0)
    {}

    Vector2 dir;
    double dist;
};


struct ShapeGeometry
{
    ShapeGeometry::ShapeGeometry(const std::vector<Vector2>& poly, const std::vector<Line>& lines):
        polygon(poly),
        lines(lines)
    {}

    ShapeGeometry::ShapeGeometry(){}

    std::vector<Vector2> polygon;
    std::vector<Line>    lines;
};


class Shape
{
public:
    Shape();
    ~Shape();

    bool init(const std::vector<Vector2>& points);

    void setDefaultNormalDirection(Vector2 defaultNormalDirection) { m_defaultNormalDirection = defaultNormalDirection; }
    void setCollisionMargin(double margin);

    AABB calcLocalAABB() const;
    AABB calcTransformedAABBWithMargin(Transform trans) const;
    double calcInertia() const;
    double calcArea() const;

    const std::vector<Vector2>& localPoints() const { return m_shapeGeometry.polygon; }

    bool overlaps(Vector2 point);
    bool overlap(const Shape& otherShape, const Transform& trans, std::vector<Contact>* contacts) const;

    std::vector<Vector2> transformedPoints(const Transform& trans) const;

    Vector2 defaultNormal() const { return m_defaultNormalDirection; }
    double collisionMargin() const { return m_collisionMargin; }

private:

    void     center();
    Vector2  determineCentroid(std::vector<Vector2> poly);

    //Private static functions could all go to anonymous namespace, but are kept here for overview and unit testing purposes.

    //Determines axis of minimum penetration, the point with maximum penetration distance, and the maximum penetration
    static bool                    sat(const ShapeGeometry& shape0, const ShapeGeometry& shape1, double totMargin, Vector2* axis, double* penetration, Vector2* pMaxPen);

    static double                  minDist(const std::vector<double>& projection0, const std::vector<double>& projection1, bool lines0, int* pIdx);
    static std::vector<double>     project(const Line& line, const std::vector<Vector2>& poly);

    //Functions used in overlap processing//////////////////////////////////////////////

    //Checks if point is inside poly lines.
    static bool                    isInside(const std::vector<Line>& lines, Vector2 point);
    
    //Minimum distance of point to poly line.
    static double                  distance(const std::vector<Vector2>& poly, Vector2 point);                                                    
    
    //Minimum distance of point pDist to line defined by p1 and p2
    static double                  lineDistance(const Vector2& p1, const Vector2& p2, const Vector2& pDist);                                     
    
    //Points where the two polygons cross.
    static std::vector<Vector2>    crossingPoints(const std::vector<Vector2>& poly0, const std::vector<Vector2>& poly1);                         
    
    //Crossing of two lines: line1 = (p1,p2), line2 = (p3,p4)
    static bool                    lineCrossing(const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, Vector2* crossing);  
    static std::array<Vector2, 2>  getMaxCrossingLine(const std::vector<Vector2>& crossingPoints);

    ////////////////////////////////////////////////////////////////////////////////////////////////

    //Functions used in shape construction
    static bool                    isCoLinear(const std::vector<Line>& lines, const Line& line);
    static std::vector<Line>       linesFromPolygon(const std::vector<Vector2>& poly);

    //Transform functions
    static std::vector<Line>       transformedLines(const std::vector<Line>& lines, const Transform& trans);
    static std::vector<Vector2>    transformedPoints(const std::vector<Vector2>& points, const Transform& trans);


    ShapeGeometry m_shapeGeometry;
    ShapeGeometry m_shapeGeometryWithMargin;

    //When one shape is completely inside another shape, the contact normal direction is undetermined.
    //The overlap can be resolved by moving the overlapping shape in any direction, but the direction
    //must be consistent between iteration steps. Therefore a default direction is stored with the shape.
    Vector2 m_defaultNormalDirection;

    double  m_collisionMargin; //Added to the shape to find almost collisions, improving stability and convergence.

};