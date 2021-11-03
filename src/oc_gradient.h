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

#ifndef _GRADIENT_H_
#define _GRADIENT_H_

#include "oc_array.h"
#include "oc_image.h"

namespace opencorr
{
	class Gradient2D4
	{
	protected:
		Image2D* grad_img = nullptr;

	public:
		Eigen::MatrixXf gradient_x;
		Eigen::MatrixXf gradient_y;
		Eigen::MatrixXf gradient_xy;

		Gradient2D4(Image2D& image);
		~Gradient2D4();

		void getGradientX(); //create an array of gradient_x
		void getGradientY(); //create an array of gradient_y
		void getGradientXY(); //create an array of gradient_xy

	};

}//namespace opencorr

#endif //_GRADIENT_H_