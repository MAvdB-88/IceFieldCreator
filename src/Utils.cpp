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

#include "Utils.h"
#include "Vector2.h"

#include <numeric>

namespace utils
{
IndexPair makePair(int i, int j)
{
	if (i >= j)
	{
		return (IndexPair(i, j));
	}
	else
	{
		return (IndexPair(j, i));
	}
};


Vector2 mean(const std::vector<Vector2>& poly)
{
	Vector2 sum = std::accumulate(poly.begin(), poly.end(), Vector2(0.0, 0.0));
	return sum / double(poly.size());
}

bool isConvexPolygon(const std::vector<Vector2>& poly)
{
	if (poly.size() < 3)
	{
		return false;
	}

	double val(0.0);

	Vector2 pBegin = poly[0];
	Vector2 pEnd = poly[poly.size() - 1];

	if ((pEnd - pBegin).length2() > DBL_EPSILON) //Polygon not closed
	{
		Vector2 p0 = poly[poly.size() - 1];
		Vector2 p1 = poly[0];
		Vector2 p2 = poly[1];

		Vector2 v0 = p1 - p0;
		Vector2 v1 = p2 - p1;

		val = v0.cross(v1);
		
		p0 = poly[poly.size() - 2];
		p1 = poly[poly.size() - 1];
		p2 = poly[0];

		v0 = p1 - p0;
		v1 = p2 - p1;

		double val2 = v0.cross(v1);

		if (val * val2 < DBL_EPSILON) //val and val2 have different signs
		{
			return false;
		}

	}
	else //Fist and last point are the same
	{
		if (poly.size() < 4)
		{
			return false;
		}

		Vector2 p0 = poly[poly.size() - 2];
		Vector2 p1 = poly[0];
		Vector2 p2 = poly[1];

		Vector2 v0 = p1 - p0;
		Vector2 v1 = p2 - p1;

		val = v0.cross(v1);

		p0 = poly[poly.size() - 3];
		p1 = poly[poly.size() - 2];
		p2 = poly[0];

		v0 = p1 - p0;
		v1 = p2 - p1;

		double val2 = v0.cross(v1);

		if (val * val2 < DBL_EPSILON) //val and val2 have different signs
		{
			return false;
		}
	}


	for (int i = 0; i < poly.size() - 2; i++)
	{
		Vector2 p0 = poly[i];
		Vector2 p1 = poly[i + size_t(1)];
		Vector2 p2 = poly[i + size_t(2)];

		Vector2 v0 = p1 - p0;
		Vector2 v1 = p2 - p1;

		double val2 = v0.cross(v1);


		if (val * val2 < DBL_EPSILON) //val and val2 have different signs
		{
			return false;
		}
	}

	return true;
}
}