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

#include <algorithm>

#include "Contact.h"
#include "Shape.h"
#include "Vector2.h"
#include "Transform.h"
#include "Utils.h"
#include "ErrorLogger.h"

#include "AABB.h"

#pragma once



namespace
{

inline double det(double a, double b, double c, double d)
{
    return a * d - b * c;
}

struct sortClockwise //Should only be used for convex polygon!
{
    bool operator()(const Vector2 p1, const Vector2 p2) const
    {
        double v1 = atan2(p1.getX(), p1.getY());
        double v2 = atan2(p2.getX(), p2.getY());

        return v1 < v2;
    }
};

}


Shape::Shape() :
    m_collisionMargin(0.0) //Added to the shape to find almost collisions, improving stability and convergence.
{
    m_defaultNormalDirection = Vector2((double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
    m_defaultNormalDirection.normalize();
}

Shape::~Shape() = default;


bool Shape::init(const std::vector<Vector2>&points)
{
    if (points.size() < 3)
    {
        ErrLog::log("Error: There must be at least three points in a shape.");
        return false;
    }

    if (!utils::isConvexPolygon(points))
    {
        ErrLog::log("Error: Shape is non-convex, IceFieldCreator currently only supports convex shapes.");
        return false;
    }

    m_shapeGeometry.polygon = points;

    Vector2 pBegin = *m_shapeGeometry.polygon.begin();
    Vector2 pEnd = m_shapeGeometry.polygon.back();

    if ((pEnd - pBegin).length2() > DBL_EPSILON) //Polygon not closed
    {
        m_shapeGeometry.polygon.push_back(pBegin);
    }

    //Check orientation:
    Vector2 v0 = m_shapeGeometry.polygon[1] - m_shapeGeometry.polygon[0];
    Vector2 v1 = m_shapeGeometry.polygon[2] - m_shapeGeometry.polygon[1];

    if (v0.cross(v1) > 0)
    {
        std::reverse(m_shapeGeometry.polygon.begin(), m_shapeGeometry.polygon.end()); //Orient counter clockwise.
    }

    //Center points around centroid
    center();

    //Init poly with margin:
    m_shapeGeometryWithMargin.polygon.resize(m_shapeGeometry.polygon.size());
    for (int i = 0; i < m_shapeGeometry.polygon.size(); i++)
    {
        const Vector2 p = m_shapeGeometry.polygon[i];
        m_shapeGeometryWithMargin.polygon[i] = p + p.normalized() * m_collisionMargin;
    }

    //Construct lines:
    m_shapeGeometry.lines = linesFromPolygon(m_shapeGeometry.polygon);
    m_shapeGeometryWithMargin.lines = linesFromPolygon(m_shapeGeometryWithMargin.polygon);

    return true;
}


void Shape::center()
{
    Vector2 centroid = determineCentroid(m_shapeGeometry.polygon);
    for (auto& element : m_shapeGeometry.polygon)
        element -= centroid;
}


void Shape::setCollisionMargin(double margin)
{
    m_collisionMargin = margin;

    //Init poly with margin:
    m_shapeGeometryWithMargin.polygon.resize(m_shapeGeometry.polygon.size());
    for (int i = 0; i < m_shapeGeometry.polygon.size(); i++)
    {
        const Vector2 p = m_shapeGeometry.polygon[i];
        m_shapeGeometryWithMargin.polygon[i] = p + p.normalized() * m_collisionMargin;
    }

    m_shapeGeometryWithMargin.lines = linesFromPolygon(m_shapeGeometryWithMargin.polygon);
}


std::vector<Vector2> Shape::transformedPoints(const Transform& trans) const
{
    return transformedPoints(m_shapeGeometry.polygon, trans);
}


bool Shape::overlap(const Shape& otherShape, const Transform& trans, std::vector<Contact>* contacts) const
{
    ShapeGeometry locGeometryOther;
    locGeometryOther.polygon = transformedPoints(otherShape.m_shapeGeometry.polygon, trans);
    locGeometryOther.lines = transformedLines(otherShape.m_shapeGeometry.lines, trans);

    ShapeGeometry locGeometryOtherMargin;
    locGeometryOtherMargin.polygon = transformedPoints(otherShape.m_shapeGeometryWithMargin.polygon, trans);
    locGeometryOtherMargin.lines = transformedLines(otherShape.m_shapeGeometryWithMargin.lines, trans);

    double totMargin = m_collisionMargin + otherShape.m_collisionMargin;

    Vector2 normal, cp;
    double penetration(0.0);
    if (!sat(m_shapeGeometry, locGeometryOther, totMargin, &normal, &penetration, &cp))
    {
        return false;
    }

    std::vector<Vector2> crossingPtsMargin = crossingPoints(m_shapeGeometryWithMargin.polygon, locGeometryOtherMargin.polygon);

    if (crossingPtsMargin.size() < 2)
    {
        //Check for one shape inside the other:
        if (isInside(locGeometryOther.lines, m_shapeGeometry.polygon[0]))
        {
            //This shape in other shape
            Vector2 normal = rotateVec(trans.orientation(), otherShape.defaultNormal());
            Vector2 centroid(0.0, 0.0);
            contacts->push_back(Contact(normal, centroid, penetration));
            return true;
        }

        if (isInside(m_shapeGeometry.lines, locGeometryOther.polygon[0]))
        {
            //Other shape in this shape
            Vector2 normal = -m_defaultNormalDirection;
            Vector2 centroid = trans.position();
            contacts->push_back(Contact(normal, centroid, penetration));
            return true;
        }

        return false; //No overlap
    }

    //Make sure the normal is always pointing towards body 0:
    if (trans.position().dot(normal) > 0) normal = -normal;

    std::array<Vector2, 2> crLineMargin = getMaxCrossingLine(crossingPtsMargin);


    const double l2 = (crLineMargin[1] - crLineMargin[0]).length2();
    const double t = (cp - crLineMargin[0]).dot(crLineMargin[1] - crLineMargin[0]) / l2;

    if (t < 0.0 || t > 1.0)
    {
        cp = 0.5 * (crLineMargin[1] + crLineMargin[0]);
    }

    contacts->push_back(Contact(normal, cp, penetration));
    {
        double dist0 = distance(locGeometryOther.polygon, crLineMargin[0]);
        double dist1 = distance(m_shapeGeometry.polygon, crLineMargin[0]);
        double distTot = dist0 + dist1;

        if (penetration > 0.0 && distTot > penetration)
        {
            distTot = penetration;
        }

        contacts->push_back(Contact(normal, crLineMargin[0], -distTot));
    }

    {
        double dist0 = distance(locGeometryOther.polygon, crLineMargin[1]);
        double dist1 = distance(m_shapeGeometry.polygon, crLineMargin[1]);
        double distTot = dist0 + dist1;

        if (penetration > 0.0 && distTot > penetration)
        {
            distTot = penetration;
        }

        contacts->push_back(Contact(normal, crLineMargin[1], -distTot));
    }

    return true;
}


double Shape::calcInertia() const
{
    const std::vector<Vector2>& poly = m_shapeGeometry.polygon;

    double k_inv3 = 1.0f / 3.0f;
    double inertia(0.0);
    for (int i = 0; i < poly.size() - 1; ++i)
    {
        // Triangle vertices.
        Vector2 p0 = poly.at(i);
        Vector2 p1 = poly.at(i + size_t(1));

        double D = p0.cross(p1);

        double intx2 = p0.x() * p0.x() + p1.x() * p0.x() + p1.x() * p1.x();
        double inty2 = p0.y() * p0.y() + p1.y() * p0.y() + p1.y() * p1.y();

        inertia += D * (intx2 + inty2);
    }

    inertia /= -12.0;

    return inertia;
}


double Shape::calcArea() const
{
    const std::vector<Vector2>& poly = m_shapeGeometry.polygon;

    double area(0.0);
    Vector2 center(0.0, 0.0);
    for (int i = 0; i < poly.size() - 1; ++i)
    {
        area += 0.5 * -poly[i].cross(poly[i + size_t(1)]);
    }

    return area;
}


bool Shape::overlaps(Vector2 point)
{
    return isInside(m_shapeGeometry.lines, point);
}


AABB Shape::calcLocalAABB() const
{
    const std::vector<Vector2>& poly = m_shapeGeometry.polygon;

    //btVector3 min(HUGE_VAL, HUGE_VAL, HUGE_VAL), max(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL);
    Vector2 min(HUGE_VAL, HUGE_VAL), max(-HUGE_VAL, -HUGE_VAL);

    //Find and return the plane's AABB:
    for (int i = 0; i < poly.size(); ++i)
    {
        if (poly[i].x() > max[0]) max[0] = poly[i].x();
        if (poly[i].y() > max[1]) max[1] = poly[i].y();
        if (poly[i].x() < min[0]) min[0] = poly[i].x();
        if (poly[i].y() < min[1]) min[1] = poly[i].y();
    }

    return AABB(min, max);
}


AABB Shape::calcTransformedAABBWithMargin(Transform trans) const
{
    const std::vector<Vector2>& poly = m_shapeGeometryWithMargin.polygon;

    Vector2 min(HUGE_VAL, HUGE_VAL), max(-HUGE_VAL, -HUGE_VAL);

    for (int i = 0; i < poly.size() - 1; ++i)
    {
        Vector2 p = poly[i];
        Vector2 pTrans = transformVec(trans, p);
        if (pTrans.x() < min.x()) min.setX(pTrans.x());
        if (pTrans.y() < min.y()) min.setY(pTrans.y());
        if (pTrans.x() > max.x()) max.setX(pTrans.x());
        if (pTrans.y() > max.y()) max.setY(pTrans.y());
    }

    return AABB(min, max);
}


//Private static member functions:

Vector2 Shape::determineCentroid(std::vector<Vector2> poly)
{
    //Function private to shape: poly size checking performed upon initialization.

    Vector2 mean = utils::mean(poly);

    for (auto& element : poly)
        element -= mean;

    std::sort(poly.begin(), poly.end(), sortClockwise());

    //Calculate centroid:
    const float k_inv3 = 1.0f / 3.0f;

    //Make sure polygon is closed
    //If polygon is already closed this will still work.
    poly.push_back(poly[0]);

    double area(0.0);
    Vector2 center(0.0, 0.0);
    for (int i = 0; i < poly.size() - 1; ++i)
    {
        // Triangle vertices.
        Vector2 e1 = poly[i];
        Vector2 e2 = poly[i + size_t(1)];

        double d = -e1.cross(e2);

        double triangleArea = 0.5 * d;
        area += triangleArea;

        // Area weighted centroid
        center += triangleArea * k_inv3 * (e1 + e2);
    }

    if (area > DBL_EPSILON)
    {
        center /= area;
        center += mean;
    }
    else
    {
        center = mean;
    }

    return center;
}


std::vector<Line> Shape::transformedLines(const std::vector<Line>&lines, const Transform & trans)
{
    int nl = lines.size();

    std::vector<Line> transLines(nl);
    for (int i = 0; i < nl; i++)
    {
        const Line& line = lines[i];

        //Transform line dir to local:
        transLines[i].dir = rotateVec(trans.orientation(), line.dir);

        //Transform line dist to local:
        Vector2 lineP = transformVec(trans, line.dir * -line.dist);
        transLines[i].dist = -transLines[i].dir.dot(lineP);
    }

    return transLines;
}


std::vector<Vector2> Shape::transformedPoints(const std::vector<Vector2>&points, const Transform & trans)
{
    int np = points.size();

    std::vector<Vector2> transPoints(np);
    for (int i = 0; i < np; i++)
    {
        transPoints[i] = transformVec(trans, points[i]);
    }

    return transPoints;
}


std::vector<Line> Shape::linesFromPolygon(const std::vector<Vector2>& poly)
{
    //Function private to shape: poly size checking performed upon initialization.

    std::vector<Line> lines;
    lines.reserve(poly.size() - 1);

    for (int i = 0; i < poly.size() - 1; i++)
    {
        Vector2 p0 = poly[i];
        Vector2 p1 = poly[i + size_t(1)];

        if ((p1 - p0).length2() < 1.0e-6)
        {
            continue; //Handle repeat points.
        }

        Vector2 dir = -(p0 - p1).perpendicular().normalized();
        double dist = -dir.dot(p0);

        Line line = Line(dir, dist);
        if (!isCoLinear(lines, line)) //Don't add colinear lines.
        {
            lines.push_back(line);
        }
    }

    return lines;
}


std::vector<Vector2> Shape::crossingPoints(const std::vector<Vector2>&poly0, const std::vector<Vector2>&poly1)
{
    //Function private to shape: poly size checking performed upon initialization.

    std::vector<Vector2> crossingPoints; //All points in the overlap polygon

    for (int i = 0; i < poly0.size() - 1; i++)
    {
        for (int j = 0; j < poly1.size() - 1; j++)
        {
            const Vector2& p1 = poly0[i];
            const Vector2& p2 = poly0[i + size_t(1)];
            const Vector2& p3 = poly1[j];
            const Vector2& p4 = poly1[j + size_t(1)];

            Vector2 crossing;
            if (lineCrossing(p1, p2, p3, p4, &crossing))
            {
                crossingPoints.push_back(crossing);
            }
        }
    }

    return crossingPoints;
}


bool Shape::sat(const ShapeGeometry& shape0, const ShapeGeometry& shape1, double totMargin, Vector2* axis, double* penetration, Vector2* pMaxPen)
{
    //Separating axis theorem-based implementation that checks for overlap or gap distance and find the axis of minimum overlap (or maximum gap)

    const std::vector<Line>& lines0 = shape0.lines;
    const std::vector<Line>& lines1 = shape1.lines;

    const std::vector<Vector2>& poly0 = shape0.polygon;
    const std::vector<Vector2>& poly1 = shape1.polygon;

    *penetration = HUGE_VAL;

    for (int i = 0; i < lines0.size(); i++)
    {
        const Line& line = lines0[i];
        std::vector<double> projection0 = project(line, poly0);
        std::vector<double> projection1 = project(line, poly1);

        bool l0(true);
        int pIdx1(-1);
        double minOverlap = minDist(projection0, projection1, l0, &pIdx1);

        if (minOverlap < -totMargin) 
        {
            return false; //Distance is larger than the margin, no contact.
        }

        if (minOverlap < *penetration)
        {
            *pMaxPen = poly1[pIdx1];
            *penetration = minOverlap;
            *axis = line.dir;
        }
    }

    for (int i = 0; i < lines1.size(); i++)
    {
        const Line& line = lines1[i];
        std::vector<double> projection0 = project(line, poly0);
        std::vector<double> projection1 = project(line, poly1);

        bool l0(false);
        int pIdx0(-1);
        double minOverlap = minDist(projection0, projection1, l0, &pIdx0);

        if (minOverlap < -totMargin)
        {
            return false;
        }

        if (minOverlap < *penetration)
        {
            *pMaxPen = poly0[pIdx0];
            *penetration = minOverlap;
            *axis = line.dir;
        }
    }

    return true;
}


//Project points on line dir. Used in SAT.
std::vector<double> Shape::project(const Line& line, const std::vector<Vector2>& poly)
{
    std::vector<double> projection(poly.size());
    for (int i = 0; i < poly.size(); i++)
    {
        projection[i] = line.dir.dot(poly[i]);
    }

    return projection;
}

//Finds the minimum overlapping distance. Used in SAT
double Shape::minDist(const std::vector<double>& projection0, const std::vector<double>& projection1, bool projectedOnLinesOfShape0, int* idxPMaxPen)
{
    //Check overlap:
    const auto min0 = std::min_element(projection0.begin(), projection0.end());
    const auto max0 = std::max_element(projection0.begin(), projection0.end());

    const auto min1 = std::min_element(projection1.begin(), projection1.end());
    const auto max1 = std::max_element(projection1.begin(), projection1.end());

    double dist;
    if (*max0 >= *min1 && *min0 <= *max1)
    {
        if (*max1 >= *max0)
        {
            dist = *max0 - *min1;

            if (projectedOnLinesOfShape0) *idxPMaxPen = min1 - projection1.begin();
            else *idxPMaxPen = max0 - projection0.begin();
        }
        else
        {
            dist = *max1 - *min0;

            if (projectedOnLinesOfShape0) *idxPMaxPen = max1 - projection1.begin();
            else *idxPMaxPen = min0 - projection0.begin();
        }
    }
    else if (*max0 <= *min1)
    {
        dist = *max0 - *min1;

        if (projectedOnLinesOfShape0) *idxPMaxPen = min1 - projection1.begin();
        else *idxPMaxPen = max0 - projection0.begin();

    }
    else
    {
        dist = *max1 - *min0;

        if (projectedOnLinesOfShape0) *idxPMaxPen = max1 - projection1.begin();
        else *idxPMaxPen = min0 - projection0.begin();

    }

    return dist;
}


//If there are more than 2 crossing points, find the subsequent points that results in the longest contact line.
std::array<Vector2, 2> Shape::getMaxCrossingLine(const std::vector<Vector2>& crossingPoints)
{
    if (crossingPoints.size() == 2)
    {
        return std::array<Vector2, 2>{crossingPoints[0], crossingPoints[1]};
    }
    else
    {
        double maxDistSq(0.0);
        std::array<Vector2, 2> crPoints;

        for (int i = 0; i < crossingPoints.size() - 1; i++)
        {
            Vector2 p0 = crossingPoints[i];
            Vector2 p1 = crossingPoints[i + 1];

            double dsq = (p1 - p0).length2();
            if (dsq > maxDistSq)
            {
                crPoints[0] = p0;
                crPoints[1] = p1;
                maxDistSq = dsq;
            }
        }

        Vector2 p0 = crossingPoints.back();
        Vector2 p1 = crossingPoints[0];
        double dsq = (p1 - p0).length2();
        if (dsq > maxDistSq)
        {
            maxDistSq = dsq;
            crPoints[0] = p0;
            crPoints[1] = p1;
        }

        return crPoints;
    }
}


bool Shape::lineCrossing(const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, Vector2* crossing)
{
    //after https://rosettacode.org/wiki/Find_the_intersection_of_two_lines#C.2B.2B

    //check line bounding boxes to see if overlap is possible.
    if (std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x())) return false;
    if (std::min(p1.x(), p2.x()) > std::max(p3.x(), p4.x())) return false;
    if (std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y())) return false;
    if (std::min(p1.y(), p2.y()) > std::max(p3.y(), p4.y())) return false;

    double detL1 = det(p1.x(), p1.y(), p2.x(), p2.y());
    double detL2 = det(p3.x(), p3.y(), p4.x(), p4.y());
    double x1mx2 = p1.x() - p2.x();
    double x3mx4 = p3.x() - p4.x();
    double y1my2 = p1.y() - p2.y();
    double y3my4 = p3.y() - p4.y();

    double denom = det(x1mx2, y1my2, x3mx4, y3my4);
    if (std::abs(denom) < 1e-9) //Lines are colinear. TODO: replace hardcoded margin with something better.
    {
        return false;
    }

    double xnom = det(detL1, x1mx2, detL2, x3mx4);
    double ynom = det(detL1, y1my2, detL2, y3my4);
    double xn = xnom / denom;
    double yn = ynom / denom;

    //TODO: replace hardcoded margin with something better.
    //Check if between points:
    if (xn > std::max({ p1.x(),p2.x() }) + 1.0e-9 || xn > std::max({ p3.x(), p4.x() }) + 1.0e-9) return false;
    if (xn < std::min({ p1.x(),p2.x() }) - 1.0e-9 || xn < std::min({ p3.x(), p4.x() }) - 1.0e-9) return false;
    if (yn > std::max({ p1.y(),p2.y() }) + 1.0e-9 || yn > std::max({ p3.y(), p4.y() }) + 1.0e-9) return false;
    if (yn < std::min({ p1.y(),p2.y() }) - 1.0e-9 || yn < std::min({ p3.y(), p4.y() }) - 1.0e-9) return false;

    crossing->setX(xn);
    crossing->setY(yn);
    return true;
}


bool Shape::isInside(const std::vector<Line>& lines, Vector2 point)
{
    int numLines = lines.size();
    for (int k = 0; k < numLines; k++)
    {
        const Line& line = lines[k];

        double dist = line.dir.dot(point) + line.dist;

        if (dist > 0.0) //Positive distance means point outside line.
        {
            return false;
        }
    }
    return true;
}


double Shape::lineDistance(const Vector2& p1, const Vector2& p2, const Vector2& pDist)
{
    // Return minimum distance between line segment p1,p2 and point pDist
    const double l2 = (p1 - p2).length2();  
    if (l2 == 0.0) return (p1 - pDist).length();   // p1 == p2 case

    const double t = (pDist - p1).dot(p2 - p1) / l2;

    if (t < 0) return (pDist - p1).length(); //p1 is closest point
    if (t > 1) return (pDist - p2).length(); //p2 is closest point

    const Vector2 projection = p1 + t * (p2 - p1);  // Projection falls on the segment
    return (projection - pDist).length();
}


bool Shape::isCoLinear(const std::vector<Line>& lines, const Line& line)
{
    for (int i = 0; i < lines.size(); i++)
    {
        const Line& line0 = lines[i];

        if (line0.dir.dot(line.dir) > (1.0 - 1e-9) //TODO: replace hardcoded margin with something better.
            && std::abs(line0.dist - line.dist) < 1.0e-9)
        {
            return true;
        }
    }

    return false;
}


//Minimum distance from polygon outer line to point. Always returns positive value, also if point is inside poly!
double Shape::distance(const std::vector<Vector2>& poly, Vector2 point)
{
    double minDist(HUGE_VAL);
    for (int i = 0; i < poly.size() - 1; i++)
    {
        Vector2 p0 = poly[i];
        Vector2 p1 = poly[i + size_t(1)];
        double dist = lineDistance(p0, p1, point);
        if (dist < minDist) minDist = dist;
    }

    return minDist;
}