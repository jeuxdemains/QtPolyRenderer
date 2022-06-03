#pragma once

struct Vec2
{
	double x, y;

	Vec2 operator+(const Vec2& point)
	{
		Vec2 v;
		v.x = this->x + point.x;
		v.y = this->y + point.y;

		return v;
	}

	Vec2 operator-(const Vec2& point)
	{
		Vec2 v;
		v.x = this->x - point.x;
		v.y = this->y - point.y;

		return v;
	}

	Vec2 operator*(const Vec2& point)
	{
		Vec2 v;
		v.x = this->x * point.x;
		v.y = this->y * point.y;

		return v;
	}

	Vec2 operator*(double val)
	{
		Vec2 v;
		v.x = this->x * val;
		v.y = this->y * val;

		return v;
	}

	Vec2 operator/(const Vec2& point)
	{
		Vec2 v;
		v.x = this->x / point.x;
		v.y = this->y / point.y;

		return v;
	}

	void operator=(const Vec2& point)
	{
		this->x = point.x;
		this->y = point.y;
	}

	Vec2 Normalize()
	{
		double len = sqrt((this->x * this->x) + (this->y * this->y));
		Vec2 normalized;
		normalized.x = this->x / len;
		normalized.y = this->y / len;

		return normalized;
	}

	double Dot(const Vec2& point)
	{
		return this->x * point.x + this->y * point.y;
	}

	double Cross(const Vec2& point)
	{
		return this->x * point.y - this->y * point.x;
	}
};

typedef Vec2 Point;

struct LineSeg
{
	Vec2 p1, p2;
};

struct VecMath
{
	static double Cross2d(double x1, double y1, double x2, double y2)
	{
		return x1 * y2 - y1 * x2;
	}

	static double Cross2d(Vec2 pointA, Vec2 pointB)
	{
		return Cross2d(pointA.x, pointA.y, pointB.x, pointB.y);
	}

	static double Cross2d(LineSeg line)
	{
		return Cross2d(line.p1.x, line.p1.y, line.p2.x, line.p2.y);
	}

	static double Dot(double x1, double y1, double x2, double y2)
	{
		return x1 * x2 + y1 * y2;
	}

	static double Dot(Vec2 pointA, Vec2 pointB)
	{
		return Dot(pointA.x, pointA.y, pointB.x, pointB.y);
	}

	static void Intersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double& x, double& y)
	{
		x = Cross2d(x1, y1, x2, y2);
		y = Cross2d(x3, y3, x4, y4);
		double det = Cross2d(x1 - x2, y1 - y2, x3 - x4, y3 - y4);
		x = Cross2d(x, x1 - x2, y, x3 - x4) / det;
		y = Cross2d(x, y1 - y2, y, y3 - y4) / det;
	}

	static double PointWorldToScreen2D(double camx, double camy, double camAngle, double pointx, double pointy)
	{
		double distX = pointx - camx;
		double distY = pointy - camy;
		double pointOnScrn = distX * cos(camAngle) + distY * sin(camAngle);

		return pointOnScrn;
	}

	static double Len(const Vec2& pointA, const Vec2& pointB)
	{
		double deltaX = pointB.y - pointA.y;
		double deltaY = pointB.x - pointA.x;

		return sqrt(deltaX * deltaX + deltaY * deltaY);
	}

	static Vec2 Normal(Vec2 pointA, Vec2 pointB)
	{
		double dx = (pointB.x - pointA.x);
		double dy = (pointA.y - pointB.y);

		Vec2 normal;
		normal.x = -dy;
		normal.y = dx;

		return normal;
	}

	static LineSeg NormalLine(Vec2 p1, Vec2 p2, double NormalLen = 20)
	{
		LineSeg pn;

		int dx = p2.x - p1.x;
		int dy = p2.y - p1.y;

		int centerX = dx / 2;
		int centerY = dy / 2;

		int reversalX = 1;
		int reversalY = 1;

		pn.p1.x = p1.x + centerX;
		pn.p1.y = p1.y + centerY;
		pn.p2.x = p1.x + centerX + (NormalLen / 2) * (reversalY * -dy) / sqrt(dx * dx + dy * dy);
		pn.p2.y = p1.y + centerY + (NormalLen / 2) * (reversalX * dx) / sqrt(dx * dx + dy * dy);

		return pn;
	}

	static Vec2 Normalize(Vec2 point)
	{
		double len = sqrt((point.x * point.x) + (point.y * point.y));
		Vec2 normalized;
		normalized.x = point.x / len;
		normalized.y = point.y / len;

		return normalized;
	}


	static LineSeg Normal2D(Vec2 point1, Vec2 point2)
	{
		// dx = x2 - x1 and dy = y2 - y1, then the normals are (-dy, dx) and (dy, -dx)

		double dx = point2.x - point1.x;
		double dy = point2.y - point1.y;

		LineSeg normal;
		normal.p1.x = -dy;
		normal.p1.y = dx;
		normal.p2.x = dy;
		normal.p2.y = -dx;

		return normal;
	}

	static double AngleBetweenPoints(const Vec2& pointA, const Vec2& pointB)
	{
		double dx = pointA.x - pointB.x;
		double dy = pointA.y - pointB.y;
		double dist = sqrt(dx * dx + dy * dy);

		return atan2(dy, dx);
	}

	static int IsFrontFace(Vec2 Camera, Vec2 pointA, Vec2 pointB)
	{
		const int RIGHT = 1, LEFT = -1, ZERO = 0;

		// subtracting co-ordinates of Camera point from
		// A and B, to make Camera the origin
		pointA.x -= Camera.x;
		pointA.y -= Camera.y;
		pointB.x -= Camera.x;
		pointB.y -= Camera.y;

		// Determining cross Product
		int cross_product = pointA.x * pointB.y - pointA.y * pointB.x;

		// return RIGHT if cross product is positive
		if (cross_product > 0)
			return RIGHT;

		// return LEFT if cross product is negative
		if (cross_product < 0)
			return LEFT;

		// return ZERO if cross product is zero. 
		return ZERO;
	}

	static Vec2 ClosestPointOnLine(LineSeg line, Vec2 point)
	{
		double lineLen = VecMath::Len(line.p1, line.p2);
		double dot =
			(((point.x - line.p1.x) * (line.p2.x - line.p1.x)) +
			((point.y - line.p1.y) * (line.p2.y - line.p1.y))) /
			pow(lineLen, 2);

		Vec2 closestPoint;
		closestPoint.x = line.p1.x + (dot * (line.p2.x - line.p1.x));
		closestPoint.y = line.p1.y + (dot * (line.p2.y - line.p1.y));

		return closestPoint;
	}

	static bool IsPointOnLine(LineSeg line, Vec2 point)
	{
		double lineLen = VecMath::Len(line.p1, line.p2);
		double pointDist1 = VecMath::Len(point, line.p1);
		double pointDist2 = VecMath::Len(point, line.p2);

		double resolution = 0.1;
		double lineLenMarginHi = lineLen + resolution;
		double lineLenMarginLo = lineLen - resolution;
		double distFromLineEnds = pointDist1 + pointDist2;


		if (distFromLineEnds >= lineLenMarginLo && distFromLineEnds <= lineLenMarginHi)
			return true;

		return false;
	}
};