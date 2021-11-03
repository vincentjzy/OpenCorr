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

#include <cmath>
#include <iostream>
#include "oc_subset.h"

namespace opencorr
{
	Subset2D::Subset2D(Point2D center, int radius_x, int radius_y) {
		if (radius_x < 1 || radius_y < 1) {
			std::cerr << "Too small radius:" << radius_x << ", " << radius_y << std::endl;
		}

		this->center = center;
		this->radius_x = radius_x;
		this->radius_y = radius_y;
		width = radius_x * 2 + 1;
		height = radius_y * 2 + 1;

		eg_mat = Eigen::MatrixXf::Zero(height, width);
	}

	void Subset2D::fill(Image2D* image) {
		Point2D topleft_point(center.x - radius_x, center.y - radius_y);
		eg_mat << image->eg_mat.block(topleft_point.y, topleft_point.x, height, width);
	}

	float Subset2D::zeroMeanNorm() {
		float subset_mean = eg_mat.mean();
		eg_mat.array() -= subset_mean;
		float subset_sum = eg_mat.squaredNorm();

		return sqrtf(subset_sum);
	}

}//namespace opencorr