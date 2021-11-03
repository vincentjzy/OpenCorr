/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#pragma once

#ifndef _POINT_H_
#define _POINT_H_

#include <cmath>
#include <iostream>

namespace opencorr
{
	//Point in 2D plane
	class Point2D
	{
	public:
		float x, y;

		Point2D();
		Point2D(float x, float y);
		Point2D(int x, int y);
		~Point2D();

		float vectorNorm() const;

		friend std::ostream& operator<<(std::ostream& output, const Point2D& point);
	};

	//reload basic operators
	inline Point2D operator+(Point2D point, Point2D offset) {
		return Point2D(point.x + offset.x, point.y + offset.y);
	}

	inline Point2D operator-(Point2D point, Point2D offset) {
		return point + Point2D(-offset.x, -offset.y);
	}

	inline Point2D operator*(float factor, Point2D point) {
		return Point2D(factor * point.x, factor * point.y);
	}

	inline Point2D operator*(Point2D point, float factor) {
		return factor * point;
	}

	inline Point2D operator*(Point2D point1, Point2D point2) {
		return Point2D(point1.x * point2.x, point1.y * point2.y);
	}

	inline Point2D operator/(Point2D point1, Point2D point2) {
		return Point2D(point1.x / point2.x, point1.y / point2.y);
	}

	// Point in 3D space
	class Point3D
	{
	public:
		float x, y, z;

		Point3D();
		Point3D(float x, float y, float z);
		Point3D(int x, int y, int z);
		~Point3D();

		float vectorNorm() const;

		friend std::ostream& operator<<(std::ostream& output, const Point3D& point);
	};

	//reload basic operators
	inline Point3D operator+(Point3D point, Point3D offset) {
		return Point3D(point.x + offset.x, point.y + offset.y, point.z + offset.z);
	}

	inline Point3D operator-(Point3D point, Point3D offset) {
		return point + Point3D(-offset.x, -offset.y, -offset.z);
	}

	inline Point3D operator*(float factor, Point3D point) {
		return Point3D(factor * point.x, factor * point.y, factor * point.z);
	}

	inline Point3D operator*(Point3D point, float factor) {
		return factor * point;
	}

	inline Point3D operator*(Point3D point1, Point3D point2) {
		return Point3D(point1.x * point2.x, point1.y * point2.y, point1.z * point2.z);
	}

	inline Point3D operator/(Point3D point1, Point3D point2) {
		return Point3D(point1.x / point2.x, point1.y / point2.y, point1.z / point2.z);
	}

}//namespace opencorr

#endif //_POINT_H_