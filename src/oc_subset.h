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

#ifndef _SUBSET_H_
#define _SUBSET_H_

#include "oc_array.h"
#include "oc_image.h"
#include "oc_point.h"

namespace opencorr
{
	class Subset2D
	{
	public:
		Point2D center;
		int radius_x, radius_y;
		int height, width;

		Eigen::MatrixXf eg_mat;

		Subset2D(Point2D center, int radius_x, int radius_y);
		~Subset2D() = default;

		void fill(Image2D* image);
		float zeroMeanNorm();

	};
}//namespace opencorr

#endif //_SUBSET_H_