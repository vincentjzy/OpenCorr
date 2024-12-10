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

#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_

#include "oc_array.h"
#include "oc_image.h"
#include "oc_point.h"

namespace opencorr
{
	//2D
	class Interpolation2D
	{
	protected:
		Image2D* interp_img = nullptr;

	public:
		int width, height;
		virtual ~Interpolation2D() = default;

		virtual void prepare() = 0;
		virtual float compute(Point2D& location) = 0;
	};

	//3D
	class Interpolation3D
	{
	protected:
		Image3D* interp_img = nullptr;

	public:
		int dim_x, dim_y, dim_z;
		virtual ~Interpolation3D() = default;

		virtual void prepare() = 0;
		virtual float compute(Point3D& location) = 0;
	};

}//namespace opencorr

#endif //_INTERPOLATION_H_
