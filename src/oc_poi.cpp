/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_poi.h"

namespace opencorr
{
	//POI2D
	POI2D::POI2D(int x, int y) :Point2D(x, y)
	{
		clear();
	}

	POI2D::POI2D(float x, float y) : Point2D(x, y)
	{
		clear();
	}

	POI2D::POI2D(Point2D location) : Point2D(location)
	{
		clear();
	}

	POI2D::~POI2D() {}

	void POI2D::clear()
	{
		std::fill(std::begin(deformation.p), std::end(deformation.p), 0.f);
		std::fill(std::begin(result.r), std::end(result.r), 0.f);
		std::fill(std::begin(strain.e), std::end(strain.e), 0.f);

		subset_radius.x = 0.f;
		subset_radius.y = 0.f;
	}


	//POI2DS
	POI2DS::POI2DS(int x, int y) :Point2D(x, y)
	{
		clear();
	}

	POI2DS::POI2DS(float x, float y) : Point2D(x, y)
	{
		clear();
	}

	POI2DS::POI2DS(Point2D location) : Point2D(location)
	{
		clear();
	}

	POI2DS::~POI2DS() {}

	void POI2DS::clear()
	{
		std::fill(std::begin(deformation.p), std::end(deformation.p), 0.f);
		std::fill(std::begin(result.r), std::end(result.r), 0.f);
		std::fill(std::begin(strain.e), std::end(strain.e), 0.f);

		Point3D zero_pt;
		ref_coor = zero_pt;
		tar_coor = zero_pt;

		subset_radius.x = 0.f;
		subset_radius.y = 0.f;
	}


	//POI3D
	POI3D::POI3D(int x, int y, int z) :Point3D(x, y, z)
	{
		clear();
	}

	POI3D::POI3D(float x, float y, float z) : Point3D(x, y, z)
	{
		clear();
	}

	POI3D::POI3D(Point3D location) : Point3D(location)
	{
		clear();
	}

	POI3D::~POI3D() {}

	void POI3D::clear()
	{
		std::fill(std::begin(deformation.p), std::end(deformation.p), 0.f);
		std::fill(std::begin(result.r), std::end(result.r), 0.f);
		std::fill(std::begin(strain.e), std::end(strain.e), 0.f);

		subset_radius.x = 0.f;
		subset_radius.y = 0.f;
		subset_radius.z = 0.f;
	}

}//namespace opencorr