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

#include <omp.h>
#include "oc_gradient.h"

namespace opencorr
{
	Gradient2D4::Gradient2D4(Image2D& image) {
		grad_img = &image;
	}

	Gradient2D4::~Gradient2D4() {
	}

	void Gradient2D4::getGradientX() {
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_x = Eigen::MatrixXf::Zero(height, width);

#pragma omp parallel for
		for (int r = 0; r < height; r++) {
			for (int c = 2; c < width - 2; c++) {
				float result = 0.0f;
				result -= grad_img->eg_mat(r, c + 2) / 12.f;
				result += grad_img->eg_mat(r, c + 1) * (2.f / 3.f);
				result -= grad_img->eg_mat(r, c - 1) * (2.f / 3.f);
				result += grad_img->eg_mat(r, c - 2) / 12.f;
				gradient_x(r, c) = result;
			}
		}
	}

	void Gradient2D4::getGradientY() {
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_y = Eigen::MatrixXf::Zero(height, width);

#pragma omp parallel for
		for (int r = 2; r < height - 2; r++) {
			for (int c = 0; c < width; c++) {
				float result = 0.0f;
				result -= grad_img->eg_mat(r + 2, c) / 12.f;
				result += grad_img->eg_mat(r + 1, c) * (2.f / 3.f);
				result -= grad_img->eg_mat(r - 1, c) * (2.f / 3.f);
				result += grad_img->eg_mat(r - 2, c) / 12.f;
				gradient_y(r, c) = result;
			}
		}
	}

	void Gradient2D4::getGradientXY() {
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_xy = Eigen::MatrixXf::Zero(height, width);

		if (gradient_x.rows() != height || gradient_x.cols() != width) {
			getGradientX();
		}

#pragma omp parallel for
		for (int r = 2; r < height - 2; r++) {
			for (int c = 0; c < width; c++) {
				float result = 0.0f;
				result -= gradient_x(r + 2, c) / 12.f;
				result += gradient_x(r + 1, c) * (2.f / 3.f);
				result -= gradient_x(r - 1, c) * (2.f / 3.f);
				result += gradient_x(r - 2, c) / 12.f;
				gradient_xy(r, c) = result;
			}
		}
	}

} //namespcae opencorr