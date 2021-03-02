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
#include "AABB.h"

class Transform;
class Contact;

//Forward declare boost ring and linestring template classes:
namespace boost
{
namespace geometry
{
namespace model
{
template
<
	typename Point,
	bool ClockWise, bool Closed,
	template<typename, typename> class Container,
	template<typename> class Allocator
>
class ring;

template
<
	typename Point,
	template<typename, typename> class Container = std::vector,
	template<typename> class Allocator = std::allocator
>
class linestring;
}
}
};

typedef boost::geometry::model::ring<Vector2, false, true, std::vector, std::allocator> bgRing;
typedef boost::geometry::model::linestring<Vector2> bgLineString;

class Shape
{
public:
	Shape();
	Shape(Shape&& other); //Needed because of unique ptr with forward declared class
	~Shape();             //Needed because of unique ptr with forward declared class

	bool init(std::vector<Vector2> vec);
	void setDefaultNormalDirection(Vector2 defaultNormalDirection);
	void setCollisionMargin(double margin) { m_collisionMargin = margin; }

	bool overlaps(Vector2 point) const;

	bool overlap(const Shape& otherShape, const Transform& trans, std::vector<Contact>* contacts) const;

	double collisionMargin() const { return m_collisionMargin; }

	AABB calcLocalAABB() const;
	AABB calcTransformedAABBWithMargin(Transform trans) const;
	double calcArea() const;
	double calcInertia() const;

	std::vector<Vector2> transformedPoints(Transform trans) const;
	const std::vector<Vector2>& localPoints() const;

	Vector2 defaultNormal() const { return m_defaultNormalDirection; }
	
private:

	void center(); //Translate such that the origin is at the centroid.

	//When one shape is completely inside another shape, the contact normal direction is undetermined.
	//The overlap can be resolved by moving the overlapping shape in any direction, but the direction
	//must be consistent between iteration steps. Therefore a default direction is stored with the shape.
	Vector2 m_defaultNormalDirection;
	double  m_collisionMargin; //Added to the shape to find almost collisions, improving stability and convergence.
	
	//Ring geometry without collision margin
	std::unique_ptr<bgRing> m_ring;  //Stored as pointer to isolate the boost headers and to keep the body vector compact.
};
