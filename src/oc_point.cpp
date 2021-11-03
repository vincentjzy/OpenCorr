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

#include "oc_point.h"

namespace opencorr
{
	//Point2D
	Point2D::Point2D() {
		x = 0.f;
		y = 0.f;
	}

	Point2D::Point2D(float x, float y) {
		this->x = x;
		this->y = y;
	}

	Point2D::Point2D(int x, int y) {
		this->x = (float)x;
		this->y = (float)y;
	}

	Point2D::~Point2D() {
	}

	float Point2D::vectorNorm() const {
		return sqrtf(x * x + y * y);
	}

	std::ostream& operator<<(std::ostream& output, const Point2D& point) {
		output << point.x << "," << point.y;
		return output;
	}

	//Point3D
	Point3D::Point3D() {
		x = 0.f;
		y = 0.f;
		z = 0.f;
	}

	Point3D::Point3D(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Point3D::Point3D(int x, int y, int z) {
		this->x = (float)x;
		this->y = (float)y;
		this->z = (float)z;
	}

	Point3D::~Point3D() {
	}

	float Point3D::vectorNorm() const {
		return sqrtf(x * x + y * y + z * z);
	}

	std::ostream& operator<<(std::ostream& output, const Point3D& point) {
		output << point.x << "," << point.y << "," << point.z;
		return output;
	}

}//namespace opencorr

