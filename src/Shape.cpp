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

#include "Shape.h"
#include "Contact.h"
#include "Transform.h"
#include "utils.h"
#include "ErrorLogger.h"

#include <array>

//Haven't yet managed to turn of the warnings resulting from the boost headers.
#include "boost/geometry/core/cs.hpp"
#include "boost/geometry/geometries/ring.hpp"
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/algorithms/unique.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>

using namespace boost::geometry;

BOOST_GEOMETRY_REGISTER_POINT_2D(Vector2, double, boost::geometry::cs::cartesian, x(), y());
typedef boost::geometry::model::ring<Vector2, false, true> bgRing; // ring<Point, ClockWise, Closed, ..>
typedef boost::geometry::model::linestring<Vector2> bgLinestring;

namespace trans = boost::geometry::strategy::transform;
using boost::geometry::dsv;

Shape::Shape(Shape&& other) = default;


Shape::Shape():
	m_ring(std::make_unique<bgRing>()),
	m_collisionMargin(0.0)
{
	m_defaultNormalDirection = Vector2((double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
	m_defaultNormalDirection.normalize();
}


Shape::~Shape() = default;


bool Shape::init(std::vector<Vector2> vec)
{
	append(*m_ring, vec);
	append(*m_ring, vec[0]); // close the polygon.
	unique(*m_ring);
	correct(*m_ring); // If points are wrongly oriented with respect to their expected orientation they will be reversed.

	if (!utils::isConvexPolygon(*m_ring))
	{
		bgRing tempRing;
		convex_hull(*m_ring, tempRing);
		*m_ring = tempRing;

		ErrLog::log("Warning: Shape is non-convex and will be initialized as convex shape.");
	}

	if (!is_valid(*m_ring))
	{
		ErrLog::log("Warning: Input points are not a valid boost polygon.");
		return false;
	}

	center(); //Translate such that the origin is at the centroid.

	return true;
}


void Shape::setDefaultNormalDirection(Vector2 defaultNormalDirection)
{
	m_defaultNormalDirection = defaultNormalDirection;
	m_defaultNormalDirection.normalize();
}


void Shape::center()
{
	Vector2 cp;
	centroid(*m_ring, cp);

	trans::translate_transformer<double, 2, 2> translate(-cp.x(), -cp.y());

	bgRing temRing = *m_ring;
	m_ring->resize(0);
	boost::geometry::transform(temRing, *m_ring, translate);
}


bool Shape::overlaps(Vector2 point) const
{
	return within(point, *m_ring);
}


const std::vector<Vector2>& Shape::localPoints() const { return *m_ring; };



namespace //Contains local functions used in overlap processing
{
struct TempLocalShape
{

	TempLocalShape(const Shape& shape, const Transform& trans)
	{
		const std::vector<Vector2>& points = shape.localPoints();
		double colMargin = shape.collisionMargin();

		std::vector<Vector2> pointsWithMargin(points.size());
		std::vector<Vector2> pointsNoMargin(points.size());
		for (int i = 0; i < points.size(); i++)
		{
			const Vector2& p = points[i];
			Vector2 pointWithMargin = p + p.normalized() * colMargin;
			pointsWithMargin[i] = transformVec(trans, pointWithMargin);
			pointsNoMargin[i] = transformVec(trans, p);
		}

		append(ringMargin, pointsWithMargin);
		append(lineStringMargin, pointsWithMargin);
		append(ringNoMargin, pointsNoMargin);
		append(lineStringNoMargin, pointsNoMargin);

		defaultNormal = rotateVec(trans.orientation(), shape.defaultNormal());
	}

	TempLocalShape(const Shape& shape)
	{
		const std::vector<Vector2>& points = shape.localPoints();
		double colMargin = shape.collisionMargin();

		std::vector<Vector2> pointsWithMargin = points;
		for (int i = 0; i < pointsWithMargin.size(); i++)
		{
			Vector2& p = pointsWithMargin[i];
			p += p.normalized() * colMargin;
		}

		append(ringMargin, pointsWithMargin);
		append(lineStringMargin, pointsWithMargin);
		append(ringNoMargin, points);
		append(lineStringNoMargin, points);
		defaultNormal = shape.defaultNormal();
	}


	bgRing  ringNoMargin;
	bgRing ringMargin;
	bgLineString lineStringNoMargin;
	bgLineString lineStringMargin;
	Vector2 defaultNormal;
};

bool ringIntersection(const bgRing& ring0, const bgRing& ring1, bgRing* intersectionRing)
{
	std::vector<bgRing> overlaps;
	intersection(ring0, ring1, overlaps);
	if (overlaps.size() != 1) return false; //return empty vec, no contacts.

	*intersectionRing = overlaps[0];
	return true;
};

bool maxLengthLine(const bgLineString& lineString0, const bgLineString& lineString1, std::array<Vector2, 2>* line)
{
	//False: no two crossings.

	std::vector<Vector2> lineCrossings;
	intersection(lineString0, lineString1, lineCrossings);
	if (lineCrossings.size() < 2) return false;

	int idxMaxL0(-1), idxMaxL1(-1);

	double maxLineLengthSqrt(0.0);
	for (int i = 0; i < lineCrossings.size(); i++)
	{
		Vector2 p0 = lineCrossings[i];

		for (int j = i + 1; j < lineCrossings.size(); j++)
		{
			Vector2 p1 = lineCrossings[j];
			double length2 = (p1 - p0).length2();
			if (length2 > maxLineLengthSqrt)
			{
				maxLineLengthSqrt = length2;
				idxMaxL0 = i;
				idxMaxL1 = j;
			}
		}
	}

	line->at(0) =  lineCrossings[idxMaxL0];
	line->at(1) = lineCrossings[idxMaxL1];

	return true;
}

double maxPenetration(const bgRing& poly, Vector2 normal)
{
	double minDist(HUGE_VAL), maxDist(-HUGE_VAL);

	//Find the maximum penetration:
	for (int i = 0; i < poly.size(); i++)
	{
		Vector2 p = poly[i];
		double dist = p.dot(normal);

		if (dist > maxDist) maxDist = dist;
		if (dist < minDist) minDist = dist;
	}
	return maxDist - minDist;
}


bool overlapOneInsideOther(const TempLocalShape& shape0, const TempLocalShape& shape1, Contact* contact)
{
	double area0 = area(shape0.ringNoMargin);
	double area1 = area(shape1.ringNoMargin);

	//Use the minimum distance as penetration distance

	
	double penetration = distance(shape0.lineStringNoMargin, shape1.lineStringNoMargin);
	contact->setPenetration(penetration);

	bgRing intersectionPoly;
	if (!ringIntersection(shape0.ringNoMargin, shape1.ringNoMargin, &intersectionPoly))
	{
		return false;
	}

	Vector2 contactPoint;
	centroid(intersectionPoly, contactPoint);
	contact->setContactPoint(contactPoint);

	double overlapArea = area(intersectionPoly);

	if (std::abs(area0 - overlapArea) < 1.0e-3)
	{
		//Current shape is inside other shape
		//Use the default normal direction of the other shape
		contact->setNormal(shape1.defaultNormal);
		double penetration = maxPenetration(intersectionPoly, contact->normal());
		if (penetration > contact->penetration()) contact->setPenetration(penetration);
	}
	else
	{
		//Other shape is in current shape
		//Use the default normal direction of the current shape
		contact->setNormal(-shape0.defaultNormal);
		double penetration = maxPenetration(intersectionPoly, contact->normal());
		if (penetration > contact->penetration()) contact->setPenetration(penetration);
	}

	return true;
}
	
	
}


bool Shape::overlap(const Shape& otherShape, const Transform& trans, std::vector<Contact>* contacts) const
{
	//Return bolean is for error checking, not for indicating if there is contact!
	//Efficiency and logic flow of the function can be improved. How would the efficiency compare to SAT or GJK algorithms?
	//Sat is simple to implement and may make the code indepdent from boost.

	//Function is currently critical to performance. Focus optimization efforts here!

	TempLocalShape locOtherShape(otherShape, trans);
	TempLocalShape locThisShape(*this);

	if (!intersects(locThisShape.ringMargin, locOtherShape.ringMargin)) return true; //No contact

	bgRing interSectionWithMargin;
	if (!ringIntersection(locThisShape.ringMargin, locOtherShape.ringMargin, &interSectionWithMargin)) return false; //intersects true, but no intersection points found

	std::array<Vector2, 2> lineCrossingsMargin;
	if (!maxLengthLine(locOtherShape.lineStringMargin, locThisShape.lineStringMargin, &lineCrossingsMargin))
	{ //No (or 1, but not sure it that ever happens) line crossing, but overlap. 
		
		Contact contact;
		overlapOneInsideOther(locThisShape, locOtherShape, &contact);
		contacts->push_back(contact);
		return true; //One shape inside the other.
	}

	Vector2 line = lineCrossingsMargin[1] - lineCrossingsMargin[0];
	if (line.length2() < 1.0e-6) return true; //Overlap insufficient for a reliable determination of contact normal direction -> no contact

	line.normalize();
	Vector2 normal = line.perpendicular();

	bgRing interSectionNoMargin;

	std::array<Vector2, 2> lineCrossingsNoMargin;
	if (maxLengthLine(locThisShape.lineStringNoMargin, locOtherShape.lineStringNoMargin, &lineCrossingsNoMargin))
	{
		Vector2 lineD = lineCrossingsNoMargin[1] - lineCrossingsNoMargin[0];
		if (lineD.length2() > 1.0e-6)
		{
			//The ringintersection may return an incorrect overlap polygon if a point 
			//of one shape is on the outer line of the other. This construct seems to be doing a good job
			//in capturing these cases, but is not 100% robust and can be improved.
			if (ringIntersection(locThisShape.ringNoMargin, locOtherShape.ringNoMargin, &interSectionNoMargin))
			{
				if (area(interSectionNoMargin) > area(interSectionWithMargin))
				{
					interSectionNoMargin.resize(0);
				}
			}
		}
	}

	Vector2 contactPoint;

	double penetration(0.0);
	if (!interSectionNoMargin.size())
	{
		//No overlap without margin.

		//Set contact point to centroid of overlap poly with margin.
		centroid(interSectionWithMargin, contactPoint);

		//Negative penetration equal to -minimum distance.
		penetration = -distance(locThisShape.ringNoMargin, locOtherShape.ringNoMargin);
	}
	else
	{
		//overlap without margin

		//Set contact point to centroid of overlap poly without margin.
		centroid(interSectionNoMargin, contactPoint);

		//Positive penetration equal to maximum difference in dot products of overlap points with normal.
		penetration = maxPenetration(interSectionNoMargin, normal);
	}

	//Make sure the normal is always pointing towards body 0.
	if (contactPoint.dot(normal) > 0) normal = -normal;

	//Construct three contacts: one at the overlap centroid, and two at the line crossings:
	{
		Contact contact;
		contact.setContactPoint(contactPoint);
		contact.setNormal(normal);
		contact.setPenetration(penetration);
		contacts->push_back(contact);
	}

	double totalMargin = m_collisionMargin + otherShape.m_collisionMargin;

	{
		Contact contact;
		contact.setContactPoint(lineCrossingsMargin[0]);

		double dist0 = distance(locThisShape.ringNoMargin, lineCrossingsMargin[0]);
		double dist1 = distance(locOtherShape.ringNoMargin, lineCrossingsMargin[0]);
		double distTot = dist0 + dist1;

		contact.setNormal(normal);
		contact.setPenetration(-distTot);
		contacts->push_back(contact);
	}

	{
		Contact contact;
		contact.setContactPoint(lineCrossingsMargin[1]);
		contact.setNormal(normal);

		double dist0 = distance(locThisShape.ringNoMargin, lineCrossingsMargin[1]);
		double dist1 = distance(locOtherShape.ringNoMargin, lineCrossingsMargin[1]);
		double distTot = dist0 + dist1;

		contact.setPenetration(-distTot);
		contacts->push_back(contact);
	}

	return true;
}


AABB Shape::calcLocalAABB() const
{
	boost::geometry::model::box<Vector2> box;
	boost::geometry::envelope(*m_ring, box);

	return AABB(box.min_corner(),box.max_corner());
}


AABB Shape::calcTransformedAABBWithMargin(Transform trans) const
{
	const bgRing& poly = *m_ring;

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

	min -= Vector2(m_collisionMargin,m_collisionMargin);
	max += Vector2(m_collisionMargin, m_collisionMargin);

	return AABB(min,max);
}



double Shape::calcArea() const
{
	//Create a polygon from a linestring:
	return boost::geometry::area(*m_ring);
}


double Shape::calcInertia() const
{
	double k_inv3 = 1.0f / 3.0f;
	double inertia(0.0);
	for (int i = 0; i < m_ring->size() - 1; ++i)
	{
		// Triangle vertices.
		Vector2 p0 = m_ring->at(i);
		Vector2 p1 = m_ring->at(i+size_t(1));

		double D = p0.cross(p1);

		double intx2 = p0.x() * p0.x() + p1.x() * p0.x() + p1.x() * p1.x();
		double inty2 = p0.y() * p0.y() + p1.y() * p0.y() + p1.y() * p1.y();

		inertia += D * (intx2 + inty2);
	}

	inertia /= 12.0;

	return inertia;
}


std::vector<Vector2> Shape::transformedPoints(Transform trans) const
{
	const bgRing& ringRef = *m_ring;

	std::vector<Vector2> transFormedPoints;

	for (int i = 0; i < ringRef.size(); i++)
	{
		Vector2 p = ringRef[i];
		transFormedPoints.push_back(transformVec(trans,p));
	}

	return transFormedPoints;
}