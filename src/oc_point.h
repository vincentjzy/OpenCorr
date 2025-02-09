/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2024, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#pragma once

#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>

namespace opencorr
{
	//Point in 2D plane
	class Point2D
	{
	public:
		float x, y;

		inline Point2D()
		{
			x = 0.f;
			y = 0.f;
		}

		inline Point2D(float x, float y)
		{
			this->x = x;
			this->y = y;
		}
		inline Point2D(int x, int y)
		{
			this->x = (float)x;
			this->y = (float)y;
		}

		inline ~Point2D() {}

		inline float vectorNorm() const
		{
			return sqrt(x * x + y * y);
		}

		inline friend std::ostream& operator<<(std::ostream& output, const Point2D& point)
		{
			output << point.x << "," << point.y;
			return output;
		}
	};

	//reload basic operators
	inline Point2D operator+(Point2D point, Point2D offset)
	{
		return Point2D(point.x + offset.x, point.y + offset.y);
	}

	inline Point2D operator-(Point2D point, Point2D offset)
	{
		return point + Point2D(-offset.x, -offset.y);
	}

	inline Point2D operator*(float factor, Point2D point)
	{
		return Point2D(factor * point.x, factor * point.y);
	}

	inline Point2D operator*(int factor, Point2D point)
	{
		return float(factor) * point;
	}

	inline Point2D operator*(Point2D point, float factor)
	{
		return factor * point;
	}

	inline Point2D operator*(Point2D point, int factor)
	{
		return float(factor) * point;
	}

	//dot product
	inline float operator*(Point2D point1, Point2D point2)
	{
		return (point1.x * point2.x + point1.y * point2.y);
	}

	inline Point2D operator/(Point2D point, float factor)
	{
		return Point2D(point.x / factor, point.y / factor);
	}

	inline Point2D operator/(Point2D point, int factor)
	{
		return point / float(factor);
	}

	//cross product
	inline float operator/(Point2D point1, Point2D point2)
	{
		return (point1.x * point2.y - point1.y * point2.x);
	}

	// Point in 3D space
	class Point3D
	{
	public:
		float x, y, z;

		inline Point3D()
		{
			x = 0.f;
			y = z = 0.f;
			z = 0.f;
		}

		inline Point3D(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		inline Point3D(int x, int y, int z)
		{
			this->x = (float)x;
			this->y = (float)y;
			this->z = (float)z;
		}

		inline ~Point3D() {}

		inline float vectorNorm() const
		{
			return sqrt(x * x + y * y + z * z);
		}

		inline friend std::ostream& operator<<(std::ostream& output, const Point3D& point)
		{
			output << point.x << "," << point.y << "," << point.z;
			return output;
		}
	};

	//reload basic operators
	inline Point3D operator+(Point3D point, Point3D offset)
	{
		return Point3D(point.x + offset.x, point.y + offset.y, point.z + offset.z);
	}

	inline Point3D operator-(Point3D point, Point3D offset)
	{
		return point + Point3D(-offset.x, -offset.y, -offset.z);
	}

	inline Point3D operator*(float factor, Point3D point)
	{
		return Point3D(factor * point.x, factor * point.y, factor * point.z);
	}

	inline Point3D operator*(int factor, Point3D point)
	{
		return float(factor) * point;
	}

	inline Point3D operator*(Point3D point, float factor)
	{
		return factor * point;
	}

	inline Point3D operator*(Point3D point, int factor)
	{
		return float(factor) * point;
	}

	//dot product
	inline float operator*(Point3D point1, Point3D point2)
	{
		return (point1.x * point2.x + point1.y * point2.y + point1.z * point2.z);
	}

	inline Point3D operator/(Point3D point, float factor)
	{
		return Point3D(point.x / factor, point.y / factor, point.z / factor);
	}

	inline Point3D operator/(Point3D point, int factor)
	{
		return point / float(factor);
	}

	//cross product
	inline Point3D operator/(Point3D point1, Point3D point2)
	{
		return Point3D((point1.y * point2.z - point1.z * point2.y),
			(point1.z * point2.x - point1.x * point2.z),
			(point1.x * point2.y - point1.y * point2.x));
	}

}//namespace opencorr

#endif //_POINT_H_