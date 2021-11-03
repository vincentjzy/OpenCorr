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

#ifndef  _BICUBIC_BSPLINE_H_
#define  _BICUBIC_BSPLINE_H_

#include "oc_interpolation.h"

namespace opencorr
{
	class BicubicBspline : public Interpolation2D
	{
	public:
		BicubicBspline(Image2D& image);
		~BicubicBspline();

		void prepare();
		float compute(Point2D& location);

	private:

		float**** lookup_table = nullptr;

		const float CONTROL_MATRIX[4][4] = {
			{ 71.0f / 56.0f, -19.0f / 56.0f, 5 / 56.0f, -1.0f / 56.0f},
			{ -19.0f / 56.0f, 95.0f / 56.0f, -25 / 56.0f, 5.0f / 56.0f},
			{ 5.0f / 56.0f, -25.0f / 56.0f, 95 / 56.0f, -19.0f / 56.0f},
			{ -1.0f / 56.0f, 5.0f / 56.0f, -19 / 56.0f, 71.0f / 56.0f}
		};

		const float FUNCTION_MATRIX[4][4] = {
			{ -1.0f / 6.0f, 3.0f / 6.0f, -3.0f / 6.0f, 1.0f / 6.0f },
			{ 3.0f / 6.0f, -6.0f / 6.0f, 3.0f / 6.0f, 0.0f },
			{ -3.0f / 6.0f, 0.0f, 3.0f / 6.0f, 0.0f },
			{ 1.0f / 6.0f, 4.0f / 6.0f, 1.0f / 6.0f, 0.0f }
		};
	};

}//namespace opencorr

#endif //_BICUBIC_BSPLINE_H_
