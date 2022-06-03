#pragma once
#include "VecMath.h"
#include <math.h>
#include <qdebug.h>

struct Physics
{
	static int LineCircleCollision(LineSeg line, Vec2 circleCenter, double circleRadius)
	{
		const int X = -1;
		const int Y = 1;

		Vec2 closestPointToLine = VecMath::ClosestPointOnLine(line, circleCenter);
		bool isClosestPointOnLine = VecMath::IsPointOnLine(line, closestPointToLine);

		if (!isClosestPointOnLine)
			return false;

		double circleToPointOnLineDist = VecMath::Len(closestPointToLine, circleCenter);

		if (circleToPointOnLineDist < circleRadius)
		{
			return 1;
		}

		return 0;
	}
};